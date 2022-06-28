/*
 * trades.h
 *
 *  Created on: 17. 9. 2019
 *      Author: ondra
 */

#ifndef SRC_MAIN_TRADERS_H_
#define SRC_MAIN_TRADERS_H_
#include "../shared/scheduler.h"
#include "../shared/shared_lockable_ptr.h"
#include "../shared/worker.h"
#include "istockapi.h"
#include "mtrader.h"
#include "stats2report.h"

using ondra_shared::Worker;
using ondra_shared::shared_lockable_ptr;

using StatsSvc = Stats2Report;

class NamedMTrader: public MTrader {
public:
	NamedMTrader(IStockSelector &sel, StoragePtr &&storage, PStatSvc statsvc, const WalletCfg &wcfg, Config cfg, std::string &&name);
	void perform(bool manually);
	const std::string ident;

};

class StockSelector: public IStockSelector{
public:
	using StockMarketMap =  ondra_shared::linear_map<std::string, PStockApi, std::less<>>;
	using TemporaryStockMap = std::map<std::string, PStockApi, std::less<> >;
	using PTemporaryStockMap = ondra_shared::shared_lockable_ptr<TemporaryStockMap>;

	StockMarketMap stock_markets;
	PTemporaryStockMap temp_markets;


	StockSelector();

	void loadBrokers(const ondra_shared::IniConfig::Section &ini, bool test, int brk_timeout);
	bool checkBrokerSubaccount(const std::string &name);
	virtual PStockApi getStock(const std::string_view &stockName) const override;
	virtual void forEachStock(EnumFn fn)  const override;
	void clear();
	void housekeepingIdle(const std::chrono::system_clock::time_point &now);
	void appendSimulator();
};


class Traders {
public:

	using TMap = ondra_shared::linear_map<std::string_view, shared_lockable_ptr<NamedMTrader> >;
	TMap traders;
    StockSelector stockSelector;
	PStorageFactory &sf;
	PReport rpt;
	PPerfModule perfMod;
	std::string iconPath;

	Traders(ondra_shared::Scheduler sch,
			const ondra_shared::IniConfig::Section &ini,
			PStorageFactory &sf,
			const PReport &rpt,
			const PPerfModule &perfMod,
			std::string iconPath,
			int brk_timeout);
	Traders(const Traders &&other) = delete;
	void clear();


	TMap::const_iterator begin() const;
	TMap::const_iterator end() const;



	void addTrader(const MTrader::Config &mcfg, ondra_shared::StrViewA n);
	void removeTrader(ondra_shared::StrViewA n, bool including_state);


	void report_util(std::string_view ident, double ms);

	template<typename Fn>
	void enumTraders(Fn &&fn) const {
		for (auto k: traders) fn(std::move(k));
	}

	void resetBrokers();
	shared_lockable_ptr<NamedMTrader> find(std::string_view id) const;
	WalletCfg wcfg;


	using Utilization = std::unordered_map<std::string, std::pair<double,std::size_t> >;
	double reset_time;

	Utilization utilization;

	json::Value getUtilization(std::size_t lastUpdate) const;

	void initExternalAssets(json::Value config);

	static void resetBroker(const PStockApi &api, const std::chrono::system_clock::time_point &now);

};




#endif /* SRC_MAIN_TRADERS_H_ */
