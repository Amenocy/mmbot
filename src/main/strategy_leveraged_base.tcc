/*
 * strategy_keepvalue.cpp
 *
 *  Created on: 20. 10. 2019
 *      Author: ondra
 */

#include "strategy_leveraged_base.h"

#include <chrono>
#include <imtjson/object.h>
#include "../shared/logOutput.h"
#include <cmath>

#include "../imtjson/src/imtjson/string.h"
#include "sgn.h"


using ondra_shared::logDebug;

template<typename Calc>
std::string_view Strategy_Leveraged<Calc>::id = Calc::id;

template<typename Calc>
Strategy_Leveraged<Calc>::Strategy_Leveraged(const PCalc &calc, const PConfig &cfg, State &&st)
:calc(calc),cfg(cfg), st(std::move(st)) {}
template<typename Calc>
Strategy_Leveraged<Calc>::Strategy_Leveraged(const PCalc &calc, const PConfig &cfg)
:calc(calc),cfg(cfg) {}


template<typename Calc>
bool Strategy_Leveraged<Calc>::isValid() const {
	return st.neutral_price > 0 && st.power > 0 && st.last_price > 0 && st.bal+cfg->external_balance > 0
			&& std::isfinite(st.val) && std::isfinite(st.neutral_price) && std::isfinite(st.power) && std::isfinite(st.bal) && std::isfinite(st.last_price) && std::isfinite(st.neutral_pos);
}

template<typename Calc>
void Strategy_Leveraged<Calc>::recalcNewState(const PCalc &calc, const PConfig &cfg, State &nwst) {
	double adjbalance = std::abs(nwst.bal + cfg->external_balance) * cfg->power;
	nwst.power = calc->calcPower(nwst.last_price, adjbalance);
	recalcNeutral(calc,cfg,nwst);
	for (int i = 0; i < 100; i++) {
		nwst.power = calc->calcPower(nwst.neutral_price, adjbalance);
		recalcNeutral(calc,cfg,nwst);
	}
	nwst.val = calc->calcPosValue(nwst.power, nwst.neutral_price, nwst.last_price);
}

template<typename Calc>
PStrategy Strategy_Leveraged<Calc>::init(const PCalc &calc, const PConfig &cfg, double price, double pos, double currency, const IStockApi::MarketInfo &minfo) {
	bool futures = minfo.leverage != 0 || cfg->longonly;
	auto bal = getBalance(*cfg,futures, price, pos, currency);
	State nwst {
		/*neutral_price:*/ price,
		/*last_price */ price,
		/*position */ pos - bal.second,
		/*bal */ bal.first-cfg->external_balance,
	};
	if (nwst.bal+cfg->external_balance<= 0) {
		//we cannot calc with empty balance. In this case, use price for calculation (but this is  unreal, trading still impossible)
		nwst.bal = price;
	}
	nwst.spot = minfo.leverage == 0;
	PCalc newcalc = calc;
	if (!newcalc->isValid(minfo)) newcalc = std::make_shared<Calc>(calc->init(minfo));
	recalcNewState(newcalc, cfg,nwst);

	auto res = PStrategy(new Strategy_Leveraged (newcalc, cfg, std::move(nwst)));
	if (!res->isValid())  {
		throw std::runtime_error("Unable to initialize strategy - invalid configuration");
	}
	return res;
}




template<typename Calc>
double Strategy_Leveraged<Calc>::calcPosition(double price) const {

	double reduction = cfg->reduction>=0?2*cfg->reduction:0;
	double dynred = 0;
	if (cfg->dynred) {
		double f;
		if (st.last_price > st.neutral_price) {
			f = st.last_price / st.neutral_price- 1.0;
		} else {
			f = st.neutral_price / st.last_price - 1.0;
		}
		dynred = std::min(1.0,f * cfg->dynred);
	}
	reduction = reduction + dynred;
	double new_neutral;


	double profit = st.position * (price - st.last_price);
	{
		//NOTE: always reduce when price is going up
		//because we need to reduce risk from short (so reduce when opening short position)
		//and we reduce opened long position as well
		//
		//for inverted futures, short and long is swapped
		if (reduction && st.position != 0 && st.last_dir) {
		//	profit += st.bal - st.redbal;
			new_neutral = calcNewNeutralFromProfit(profit, price,reduction);
		} else {
			new_neutral = st.neutral_price;
		}
	}



	double pos = calc->calcPosition(st.power, new_neutral, price);
	if ((cfg->longonly || st.spot) && pos < 0) pos = 0;
	double pp = pos * st.position ;
	if (pp < 0) return 0;
/*	else if (pp == 0) {
		pos = pos * std::pow(2.0,cfg->initboost);
	}*/
	return pos;

}

template<typename Calc>
PStrategy Strategy_Leveraged<Calc>::onIdle(
		const IStockApi::MarketInfo &minfo,
		const IStockApi::Ticker &ticker, double assets, double currency) const {
	if (isValid()) {
		if (st.power <= 0) {
			State nst = st;
			recalcNewState(calc, cfg, nst);
			return new Strategy_Leveraged<Calc>(calc, cfg, std::move(nst));
		} else {
			return this;
		}
	}
	else {
		return init(calc, cfg,ticker.last, assets, currency, minfo);
	}
}

template<typename Calc>
double Strategy_Leveraged<Calc>::calcNewNeutralFromPnl(double price, double pnl) const {

	double reduction = cfg->reduction>=0?2*cfg->reduction:0;
	double dynred = 0;
	if (cfg->dynred) {
		double f;
		if (st.last_price > st.neutral_price) {
			f = st.last_price / st.neutral_price- 1.0;
		} else {
			f = st.neutral_price / st.last_price - 1.0;
		}
		dynred = std::min(1.0,f * cfg->dynred);
	}
	reduction = reduction + dynred;

	double sel_value;
	double worst_value = calc->calcPosValue(st.power, st.neutral_price, price);
	double best_value = st.val-pnl;
	if (reduction == 0 && dynred == 0) {
		double sprd = price/st.last_price;
		double refval = calc->calcPosValue(st.power, st.neutral_price, st.neutral_price *sprd);
		sel_value = best_value+refval;
	} else {
		sel_value = worst_value + (best_value-worst_value)*reduction;
	}
	if (sel_value <= 0) return price;
	double new_neutral = calc->calcNeutralFromValue(st.power, st.neutral_price, sel_value, price);
	if (std::abs(st.neutral_price-price)<std::abs(new_neutral-price)) new_neutral = st.neutral_price;
	return new_neutral;
}

template<typename Calc>
double Strategy_Leveraged<Calc>::calcNewNeutralFromProfit(double profit, double price, double reduction) const {

	double middle = calc->calcPrice0(st.neutral_price);
	if ((middle - st.last_price ) * (middle - price) <= 0)
		return st.neutral_price;


	double new_val;
	bool rev_shift = ((price >= middle && price <= st.neutral_price) || (price <= middle && price >= st.neutral_price));
	double prev_val = st.val;
	new_val = prev_val - profit;
	double c_neutral;
	double neutral_from_price = calc->calcNeutralFromPrice0(price);
	if (calc->calcPosValue(st.power,  neutral_from_price, price) > new_val) {
		c_neutral = neutral_from_price;
	} else {
		c_neutral = calc->calcNeutralFromValue(st.power, st.neutral_price, new_val, price);
		if (rev_shift) {
			c_neutral = 2*st.neutral_price - c_neutral;
		}
	}

	double final_reduction;
	if (reduction <= 0.5) {
		if (profit > 0) final_reduction = reduction * 2; else final_reduction = 0;
	} else if (reduction <= 1 ){
		if (profit > 0) final_reduction = 1; else final_reduction = 2*(reduction-0.5);
	} else {
		if (profit >= 0) final_reduction = 2*reduction-1; else final_reduction = 1;
	}

	double new_neutral = st.neutral_price + (c_neutral - st.neutral_price) * (final_reduction);
	if ((new_neutral - price)*(st.neutral_price-price) < 0) new_neutral = price;
	return new_neutral;
}

template<typename Calc>
void Strategy_Leveraged<Calc>::recalcPower(const PCalc &calc, const PConfig &cfg, State &nwst) {
	double offset = calc->calcPosition(nwst.power, nwst.neutral_price, nwst.neutral_price);
	double adjbalance = std::abs(nwst.bal  + cfg->external_balance + nwst.neutral_price * std::abs(nwst.position - offset) * cfg->powadj) * cfg->power;
	double power = calc->calcPower(nwst.neutral_price, adjbalance);
	if (std::isfinite(power)) {
		nwst.power = power;
	}
}

template<typename Calc>
void Strategy_Leveraged<Calc>::recalcNeutral(const PCalc &calc, const PConfig &cfg,State &nwst)  {
	double neutral_price = calc->calcNeutral(nwst.power, nwst.position,
			nwst.last_price);
	if (std::isfinite(neutral_price) && neutral_price > 0) {
		nwst.neutral_price = neutral_price;
	}
}

template<typename Calc>
std::pair<typename Strategy_Leveraged<Calc>::OnTradeResult, PStrategy> Strategy_Leveraged<Calc>::onTrade(
		const IStockApi::MarketInfo &minfo,
		double tradePrice, double tradeSize, double assetsLeft,
		double currencyLeft) const {


	if (!isValid()) {
		return init(calc, cfg,tradePrice, assetsLeft, currencyLeft, minfo)
				->onTrade(minfo, tradePrice, tradeSize, assetsLeft, currencyLeft);
	}
	assetsLeft = roundZero(assetsLeft, minfo, tradePrice);

	State nwst = st;
	double apos = assetsLeft - st.neutral_pos;

	double pnl = (apos-tradeSize)*(tradePrice-st.last_price);
	nwst.neutral_price = calcNewNeutralFromPnl(tradePrice, pnl);
	if (tradeSize) recalcPower(calc, cfg, nwst);
	nwst.position = calc->calcPosition(nwst.power, nwst.neutral_price, tradePrice);
	//store last price
	nwst.last_price = tradePrice;
	nwst.last_dir = sgn(tradeSize);

	if ((cfg->longonly || st.spot) && nwst.position < 0) {
		nwst.position = 0;
		nwst.neutral_price = tradePrice;
	}

	nwst.val = calc->calcPosValue(nwst.power, nwst.neutral_price, tradePrice);
	double np = (nwst.val - st.val)+pnl;


	if (st.avgprice) {
		nwst.avgprice = std::exp((50*std::log(st.avgprice) + std::log(tradePrice))/51);
		nwst.neutral_pos = (tradePrice - st.avgprice)*cfg->trend_factor*(st.bal + cfg->external_balance)/pow2(st.avgprice);
	} else {
		nwst.avgprice = tradePrice;
		nwst.neutral_pos = 0;
	}


//	double baladj = (val - st.val) + profit;
//	double vbaladj = (val - st.val) + vprofit;

	if (cfg->reinvest_profit) {
		nwst.bal += np;
	}


	return {
		OnTradeResult{np,0,nwst.neutral_price,0},
		new Strategy_Leveraged<Calc>(calc, cfg,  std::move(nwst))
	};

}

template<typename Calc>
json::Value Strategy_Leveraged<Calc>::storeCfgCmp() const {
	return json::Object({{"ebal",
			static_cast<int>(cfg->external_balance * 1000)},{"power",
			static_cast<int>(cfg->power * 1000)},{"lo",cfg->longonly}});
}

template<typename Calc>
json::Value Strategy_Leveraged<Calc>::exportState() const {
	return json::Object({
			{"neutral_price",st.neutral_price},
			{"last_price",st.last_price},
			{"position",st.position},
			{"balance",st.bal},
			{"val",st.val},
			{"power",st.power},
			{"neutral_pos",st.neutral_pos},
			{"avgprice", st.avgprice},
			{"last_dir", st.last_dir},
			{"norm_profit", st.norm_profit},
			{"cfg", storeCfgCmp()}
			});

}

template<typename Calc>
PStrategy Strategy_Leveraged<Calc>::importState(json::Value src,const IStockApi::MarketInfo &minfo) const {
		State newst {
			src["neutral_price"].getNumber(),
			src["last_price"].getNumber(),
			src["position"].getNumber(),
			src["balance"].getNumber(),
			src["val"].getNumber(),
			src["norm_profit"].getNumber(),
			src["power"].getNumber(),
			src["neutral_pos"].getNumber(),
			src["avgprice"].getNumber(),
			src["last_dir"].getInt(),
			minfo.leverage == 0
		};
		json::Value cfgcmp = src["cfg"];
		json::Value cfgcmp2 = storeCfgCmp();
		if (cfgcmp != cfgcmp2) {
			double last_price = newst.last_price;
			if (cfg->recalc_keep_neutral) {
				newst.last_price = calc->calcPrice0(newst.neutral_price);
				newst.position = 0;
				recalcNewState(calc, cfg,newst);
				newst.last_price = last_price;
				newst.position = calc->calcPosition(newst.power,newst.neutral_price, last_price);
			} else {
				recalcNewState(calc, cfg,newst);
			}
			newst.val= calc->calcPosValue(newst.power,newst.neutral_price, last_price);
		}
		PCalc newcalc = calc;
		if (!newcalc->isValid(minfo)) newcalc = std::make_shared<Calc>(newcalc->init(minfo));
		return new Strategy_Leveraged<Calc>(newcalc, cfg, std::move(newst));
}

template<typename Calc>
double Strategy_Leveraged<Calc>::roundZero(double finpos,
		const IStockApi::MarketInfo &minfo, double price) const {
	double afinpos = std::abs(finpos);
	if (afinpos < minfo.min_size || afinpos < minfo.min_volume / price
			|| afinpos < (st.bal + cfg->external_balance) * 1.0e-10/price) {
		finpos = 0;
	}
	return finpos;
}

template<typename Calc>
IStrategy::OrderData Strategy_Leveraged<Calc>::getNewOrder(
		const IStockApi::MarketInfo &minfo,
		double curPrice, double price, double dir, double assets, double currency, bool rej) const {
	auto apos = assets - st.neutral_pos;
	double bal = (st.bal+cfg->external_balance);;
	double lev = std::abs(st.position) * st.last_price / bal;
	if (!rej && ((lev > 0.5 && dir != st.last_dir) || lev>2)  && st.val > 0) {
		if (cfg->fastclose && dir * st.position < 0) {
			double midl = calc->calcPrice0(st.neutral_price);
			double calc_price = (price - midl) * (st.last_price - midl) < 0?midl:price;
			double newval = calc->calcPosValue(st.power, st.neutral_price, calc_price);
			double valdiff = st.val - newval;
			if (valdiff > 0) {
				double fastclose_delta = valdiff/st.position;
				double close_price = fastclose_delta+st.last_price;
				if (close_price * dir < curPrice * dir && close_price * dir > price * dir) {
					price = close_price;
				}
			}
		}
		if (cfg->slowopen && dir * st.position > 0) {
			double newval = calc->calcPosValue(st.power, st.neutral_price, price);
			double valdiff = newval - st.val;
			if (valdiff > 0) {
				double delta = -valdiff/st.position;
				double open_price = delta + st.last_price;
				if (open_price * dir < price * dir &&  price > 0) {
					logDebug("Slow open active: valdiff=$1, delta=$2, spread=$3",
							valdiff, delta, price-st.last_price);
					price = open_price;
				}

			}
		}

	}

	double pnl = (price - st.last_price) * apos;
	double apsz = roundZero(apos, minfo, price);
	double limit = cfg->open_limit?((apsz == 0?(cfg->external_balance+st.bal)/price:std::abs(apsz))*(1.0+1.0/cfg->open_limit)):std::numeric_limits<double>::max();
	double new_neutral = apsz?calcNewNeutralFromPnl(price, pnl):st.neutral_price;
	double cps = calc-> calcPosition(st.power, new_neutral, price);
	double cpsz = roundZero(cps, minfo, price);
	if (!rej &&  cpsz * apsz  < 0) {
		return {0,-apos,Alert::stoploss};
	}

	double df = cps - apos;
	double eq = getEquilibrium(assets);
	double pdif = price - st.last_price;
	if (df * dir < 0 && cpsz) {
		double cps2 = calc->calcPosition(st.power, new_neutral, eq+pdif);
		double df2 = cps2 - apos;
		df = df2*pow2(cps)/pow2(std::abs(cps)+std::abs(cps2));
	} else if (st.last_dir && st.last_dir != dir) {
		double cps2 = calc->calcPosition(st.power, new_neutral, eq+pdif);
		double df2 = cps2 - apos;
		if (dir * df2 > 0) df = df2;
	}
	if (!std::isfinite(df))
		return {0,0,Alert::forced};
	double finpos = assets+df;
	if (std::abs(finpos) > limit) finpos = sgn(finpos) * limit;
/*	double curPos = roundZero(apos, minfo, price);
	if (curPos == 0 && cfg->initboost) {
		double maxpos = 0.5*(st.bal+cfg->external_balance)/price;
		finpos = finpos * std::pow(2.0,cfg->initboost);
		if (finpos * dir > maxpos) finpos = maxpos * dir;
	}*/
	bool longonly = (cfg->longonly || st.spot);
	if (longonly && roundZero(finpos, minfo, price) <= 0 ) finpos = 0;
	auto alert = finpos == 0?Alert::forced:Alert::enabled;
	df = finpos - assets;
	return {price, df,  alert};
}

template<typename Calc>
typename Strategy_Leveraged<Calc>::MinMax Strategy_Leveraged<Calc>::calcSafeRange(
		const IStockApi::MarketInfo &minfo,
		double assets,
		double currencies) const {

	auto r = calcRoots();
	if (cfg->longonly || st.spot) {
		r.max = st.neutral_price;
	}
	return r;
}

template<typename Calc>
double Strategy_Leveraged<Calc>::getEquilibrium(double assets) const {
	return  calc->calcPriceFromPosition(st.power, st.neutral_price, assets-st.neutral_pos);
}

template<typename Calc>
PStrategy Strategy_Leveraged<Calc>::reset() const {
	return new Strategy_Leveraged<Calc>(calc, cfg,{});
}

template<typename Calc>
json::Value Strategy_Leveraged<Calc>::dumpStatePretty(
		const IStockApi::MarketInfo &minfo) const {

	double bal = cfg->external_balance+st.bal;
	return json::Object({{"Position", (minfo.invert_price?-1:1)*st.position},
				  {"Last price", minfo.invert_price?1/st.last_price:st.last_price},
				 {"Neutral price", minfo.invert_price?1/st.neutral_price:st.neutral_price},
				 {"Value", st.val},
				 {"Collateral", bal},
				 {"Current leverage",  std::abs(st.position) * st.last_price / bal},
				 {"Multiplier", st.power},
				 {"Neutral pos", (minfo.invert_price?-1:1)*st.neutral_pos},
				 {"Norm profit", st.norm_profit},
	 	 	 	 {"Avg price", minfo.invert_price?1/st.avgprice:st.avgprice}});


}




template<typename Calc>
typename Strategy_Leveraged<Calc>::MinMax Strategy_Leveraged<Calc>::calcRoots() const {
	if (!rootsCache.has_value()) {
		rootsCache = calc->calcRoots(st.power,st.neutral_price, st.bal+cfg->external_balance);
	}
	return *rootsCache;
}



template<typename Calc>
std::pair<double,double> Strategy_Leveraged<Calc>::getBalance(const Config &cfg, bool leveraged, double price, double assets, double currency) {
	if (leveraged) {
		if (cfg.external_balance) return {cfg.external_balance, 0};
		else return {currency, 0};
	} else {
		double md = assets + currency / price;
		double bal = cfg.external_balance?cfg.external_balance:(md * price);
		return {bal, 0};
	}
}

template<typename Calc>
inline double Strategy_Leveraged<Calc>::calcInitialPosition(const IStockApi::MarketInfo &minfo, double price, double assets, double currency) const {
	if (!isValid()) {
		return init(calc, cfg, price, assets, currency, minfo)->calcInitialPosition(minfo,price,assets,currency);
	} else {
		return calcPosition(st.neutral_price);
	}
}

template<typename Calc>
typename Strategy_Leveraged<Calc>::BudgetInfo Strategy_Leveraged<Calc>::getBudgetInfo() const {
	return {st.bal + cfg->external_balance, 0};
}


template<typename Calc>
inline double Strategy_Leveraged<Calc>::calcCurrencyAllocation(double,bool) const {
	return cfg->external_balance + st.bal - st.val - (st.spot?st.position*st.last_price:0.0);
}

template<typename Calc>
typename Strategy_Leveraged<Calc>::ChartPoint Strategy_Leveraged<Calc>::calcChart(double price) const {
	return {true,calc->calcPosition(st.power, st.neutral_price, price), cfg->external_balance+st.bal-calc->calcPosValue(st.power,  st.neutral_price, price)};
}

