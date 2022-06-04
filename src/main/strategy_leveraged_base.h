/*
 * strategy_leveraged_base.h
 *
 *  Created on: 9. 5. 2020
 *      Author: ondra
 */

#ifndef SRC_MAIN_STRATEGY_LEVERAGED_BASE_H_
#define SRC_MAIN_STRATEGY_LEVERAGED_BASE_H_
#include <memory>




template<typename Calc>
class Strategy_Leveraged: public IStrategy {
public:

	using TCalc = Calc;
	using PCalc = std::shared_ptr<Calc>;

	struct Config {
		double power;
		double reduction = 0;
		double external_balance = 0;
		double powadj = 0;
		double dynred = 0;
		double open_limit = 0;
		double trend_factor=0;
		bool recalc_keep_neutral =false;
		bool longonly = false;
		bool fastclose = false;
		bool slowopen = false;
		bool reinvest_profit = false;
	};

	using PConfig = std::shared_ptr<const Config>;


	struct State {
		double neutral_price = 0;
		double last_price =0;
		double position = 0;
		double bal = 0;
		double val = 0;
		double norm_profit = 0;
		double power = 0;
		double neutral_pos = 0;
		double avgprice = 0;
		long last_dir = 0;
		bool spot = false;
	};

	Strategy_Leveraged(const PCalc &calc, const PConfig &cfg, State &&st);
	Strategy_Leveraged(const PCalc &calc, const PConfig &cfg);

	virtual bool isValid() const override;
	virtual PStrategy  onIdle(const IStockApi::MarketInfo &minfo, const IStockApi::Ticker &curTicker, double assets, double currency) const override;
	virtual std::pair<OnTradeResult,PStrategy > onTrade(const IStockApi::MarketInfo &minfo, double tradePrice, double tradeSize, double assetsLeft, double currencyLeft) const override;;
	virtual json::Value exportState() const override;
	virtual PStrategy importState(json::Value src,const IStockApi::MarketInfo &minfo) const override;
	virtual OrderData getNewOrder(const IStockApi::MarketInfo &minfo,  double cur_price,double new_price, double dir, double assets, double currency, bool rej) const override;
	virtual MinMax calcSafeRange(const IStockApi::MarketInfo &minfo, double assets, double currencies) const override;
	virtual double getEquilibrium(double assets) const override;
	virtual PStrategy reset() const override;
	virtual json::Value dumpStatePretty(const IStockApi::MarketInfo &minfo) const override;
	virtual std::string_view getID() const override {return id;}
	virtual BudgetInfo getBudgetInfo() const override;
	virtual double calcCurrencyAllocation(double, bool) const override;
	virtual std::optional<BudgetExtraInfo> getBudgetExtraInfo(double price, double currency) const {
		return std::optional<BudgetExtraInfo>();
	}
	virtual ChartPoint calcChart(double price) const override;
	virtual double getCenterPrice(double lastPrice, double assets) const override {return lastPrice;}



	static std::string_view id;

protected:
	PCalc calc;
	PConfig cfg;
	State st;
	mutable std::optional<MinMax> rootsCache;


	static PStrategy init(const PCalc &calc, const PConfig &cfg, double price, double pos, double currency, const IStockApi::MarketInfo &minfo);
	double calcPosition(double price) const;

	MinMax calcRoots() const;
	double adjNeutral(double price, double value) const;


	double calcNewNeutralFromProfit(double profit, double price, double reduction) const;
	double roundZero(double finpos, const IStockApi::MarketInfo &minfo,
			double price) const;

private:
	static void recalcPower(const PCalc &calc, const PConfig &cfg, State &nwst) ;
	static void recalcNeutral(const PCalc &calc, const PConfig &cfg, State &nwst) ;
	json::Value storeCfgCmp() const;
	static void recalcNewState(const PCalc &calc, const PConfig &cfg, State &nwst);
	static std::pair<double,double> getBalance(const Config &cfg, bool leveraged, double price, double assets, double currency);
	virtual double calcInitialPosition(const IStockApi::MarketInfo &minfo, double price, double assets, double currency) const override;
	double calcNewNeutralFromPnl(double price, double pnl) const;

};

#endif /* SRC_MAIN_STRATEGY_LEVERAGED_BASE_H_ */
