/*
 * istatsvc.h
 *
 *  Created on: 19. 5. 2019
 *      Author: ondra
 */

#ifndef SRC_MAIN_ISTATSVC_H_
#define SRC_MAIN_ISTATSVC_H_

#include "istatsvc.h"

#include <cmath>
#include <memory>
#include <optional>

#include "istockapi.h"

struct MTrader_Config;
struct PerformanceReport;
class Strategy;

class IStatSvc {
public:

	struct ChartItem {
		std::uint64_t time;
		double ask;
		double bid;
		double last;
	};

	struct MiscData {
		int trade_dir;
		bool achieve_mode;
		bool enabled;
		bool trade_now;
		double calc_price;
		double spread;
		double dynmult_buy;
		double dynmult_sell;
		double lowest_price;
		double highest_price;
		double budget_total;
		double budget_assets;
		double accumulated;
		std::optional<double> budget_extra;
		std::size_t total_trades;
		std::uint64_t total_time;
		double lastTradePrice;
		double position;
		double cur_norm_buy;
		double cur_norm_sell;
		double entry_price;
		double rpnl;
		double upnl;
	};


	struct Info {
		std::string_view title;
		std::string_view assetSymb;
		std::string_view currencySymb;
		std::string_view priceSymb;
		std::string_view brokerIcon;
		std::string_view brokerName;
		std::string_view walletId;
		double order;
		bool inverted;
		bool margin;
		bool emulated;
	};

	struct ErrorObj {
		std::string_view genError;
		std::string_view buyError;
		std::string_view sellError;
		explicit ErrorObj(const char *what): genError(what) {}
		ErrorObj(const std::string_view &buy_error,const std::string_view &sell_error)
			: buyError(buy_error),sellError(sell_error) {}

	};
	
	struct TradesInfo {
	    double finalPos;
	    bool inverted;
	    double total_budget;
	};

	struct TradeRecord: public IStockApi::Trade {

		double norm_profit;
		double norm_accum;
		double neutral_price;
        bool partial_exec;
		bool manual_trade = false;
		char alertSide;
		char alertReason;

		TradeRecord(const IStockApi::Trade &t, double norm_profit, double norm_accum, double neutral_price, bool partial, bool manual = false, char as = 0, char ar = 0)
			:IStockApi::Trade(t),norm_profit(norm_profit),norm_accum(norm_accum),neutral_price(neutral_price),partial_exec(partial),manual_trade(manual),alertSide(as),alertReason(ar) {}


	    static TradeRecord fromJSON(json::Value v) {
	    	double np = v["np"].getNumber();
	    	double ap = v["ap"].getNumber();
	    	double p0 = v["p0"].getNumber();
	    	bool m = v["man"].getBool();
	    	bool p = v["pe"].getBool();
	    	char alertSize = static_cast<char>(v["as"].getInt());
	    	char alertReason = static_cast<char>(v["ar"].getInt());
	    	if (!std::isfinite(np)) np = 0;
	    	if (!std::isfinite(ap)) ap = 0;
	    	if (!std::isfinite(p0)) p0 = 0;
	    	return TradeRecord(IStockApi::Trade::fromJSON(v), np, ap, p0, p, m,alertSize,alertReason);
	    }
	    json::Value toJSON() const {
	    	return IStockApi::Trade::toJSON().merge(json::Value(json::object,{
	    			json::Value("np",norm_profit),
					json::Value("ap",norm_accum),
					json::Value("p0",neutral_price),
	    			json::Value("man",manual_trade?json::Value(true):json::Value()),
                    json::Value("pe",partial_exec?json::Value(true):json::Value()),
					json::Value("as", alertSide == 0?json::Value():json::Value(alertSide)),
					json::Value("ar", alertReason == 0?json::Value():json::Value(alertReason))
	    	}));
	    }


	};

	virtual void reportOrders(int n, const std::optional<IStockApi::Order> &buy,
							  const std::optional<IStockApi::Order> &sell) = 0;
	virtual void reportTrades(const TradesInfo &tinfo, ondra_shared::StringView<TradeRecord> trades) = 0;
	virtual void reportPrice(double price) = 0;
	virtual void setInfo(const Info &info) = 0;
	virtual void reportMisc(const MiscData &miscData, bool initial = false) = 0;
	virtual void reportError(const ErrorObj &errorObj) = 0;
	virtual void reportPerformance(const PerformanceReport &repItem) = 0;
	virtual std::size_t getHash() const = 0;

	virtual ~IStatSvc() {}
};


using PStatSvc = std::unique_ptr<IStatSvc>;

#endif /* SRC_MAIN_ISTATSVC_H_ */
