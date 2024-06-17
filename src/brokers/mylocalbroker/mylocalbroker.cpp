#include "mylocalbroker.h"

#include <sstream>
#include <iomanip>
#include <random>
#include <string>
#include <string_view>
#include <thread>

#include <imtjson/object.h>

#include <imtjson/parser.h>

#include <imtjson/serializer.h>

#include <imtjson/binjson.tcc>

#include <imtjson/operations.h>

#include <shared/toString.h>

#include <simpleServer/urlencode.h>

#include "../../shared/logOutput.h"

using ondra_shared::logDebug;

using namespace json;

static std::string_view
	favicon("iVBORw0KGgoAAAANSUhEUgAAAIAAAACABAMAAAAxEHz4AAAAG1BMVEUhAAAXp4oZqI"
			"sbqYwdqo0g"
			"q44hrI8jrpAlr5HbXGoxAAAAAXRSTlMAQObYZgAAAg9JREFUaN7tmTtuxDAMRBcwME"
			"fIEVK72vMQ"
			"MMD7HyEpYmQl8atJulUpyoNn/UiRj8e71RpUL/"
			"L778Z+r+e2gP40EkBVSIBdBPwKCAmwh4BXASEB"
			"IgQ4VowCEoJeKYCP4OljFpB4plIAD+G2nhmAg+"
			"BuVtUaAhwrLAFpCKgWEZ62EbaAlM+bahXBtsET"
			"kKKAahnBNMEXkJKAah3BskwAnyGCJTBBHyGCYcD8wTNCMASWWQsR1n6swyOEVcBYtg"
			"hh6YY1OEBY"
			"BMx9EyDMvbCH+gjzWG/"
			"jughTJ7yBo+F0BbSAOhpGAfiThZKAlnbM4MgGAUR7FrZp6NPiwfUEEF8d"
			"z/QXksvryAiQ3eBIViF3Y7HAR8ORFi5gqQaudR/"
			"mhM4tgHt4wYl5AdVRdMMyYDeCSWeSpCsg0U9X"
			"BJZpv3oC0nmxxABHHqvHADDOby5QCAJDAcmDwHpIrRsCwgrQBPwc0KvA7wN6J/"
			"JngT6N/H1A30j8"
			"nRgve0WA9gu0Z9rxjbR3puMDPkJJEdgoDXmgycaJfKRKx8p8tE6/F/gXC/"
			"1m8hBQfrU5CA3vZyKg"
			"438thM7b2UJA6/"
			"VuIPTyBysCmhmMBaGbQ+GzOHQeic9k0bk0PpvX8Hh0RpPOqf5dVnc3r/zwBKqZ"
			"bT63Tmf3/VpWOfBiKxx8jeX/"
			"qjx0nYmvdNG1Nr7aR9cbb4RtAL7myld93y1uX1kIUAloS5SdAAAA"
			"AElFTkSuQmCC");

static std::string_view licence(
	R"mit(Copyright (c) 2019 Ondřej Novák

		Permission is hereby granted, free of charge, to any person
		obtaining a copy of this software and associated documentation
		files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use,
		copy, modify, merge, publish, distribute, sublicense, and/or sell
		copies of the Software, and to permit persons to whom the
		Software is furnished to do so, subject to the following
		conditions:

		The above copyright notice and this permission notice shall be
		included in all copies or substantial portions of the Software.

		THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
		EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
		OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
		NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
		HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
		WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
		FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
		OTHER DEALINGS IN THE SOFTWARE.)mit");

static Value
	apiKeyFmt({Object{{"name", "key"}, {"label", "Key"}, {"type", "string"}},
			   Object{{"name", "api_address"},
					  {"label", "api_address"},
					  {"type", "string"}}});
// https://api.nobitex.ir
MyLocalBrokerIFC::MyLocalBrokerIFC(const std::string &cfg_file)
	: AbstractBrokerAPI(cfg_file, apiKeyFmt),
	  api(simpleServer::HttpClient("TraderBot/AMINMMPRO",
								   simpleServer::newHttpsProvider(), 0,
								   simpleServer::newCachedDNSProvider(15)),
		  "https://api.nobitex.ir") {}

IBrokerControl::BrokerInfo MyLocalBrokerIFC::getBrokerInfo()
{
	return BrokerInfo{hasKey(),
					  "mylocalbroker",
					  "mylocalbroker",
					  "https://www.mylocalbroker.com/",
					  "1.0",
					  std::string(licence),
					  std::string(favicon),
					  false,
					  true};
}

std::vector<std::string> MyLocalBrokerIFC::getAllPairs()
{
	updateSymbols();
	std::vector<std::string> out;
	out.reserve(symbolMap.size());
	for (const auto &x : symbolMap)
		out.push_back(x.first);
	return out;
}

bool MyLocalBrokerIFC::areMinuteDataAvailable(
	const std::string_view &asset, const std::string_view &currency)
{
	updateSymbols();
	auto iter =
		std::find_if(symbolMap.begin(), symbolMap.end(), [&](const auto &x)
					 { return x.second.currency_symbol == currency &&
							  x.second.asset_symbol == asset; });
	return iter != symbolMap.end();
}

IStockApi::MarketInfo
MyLocalBrokerIFC::getMarketInfo(const std::string_view &pair)
{
	const auto &s = findSymbol(pair);
	if (s.fees < 0)
		updateSymbolFees(pair);
	return s;
}

AbstractBrokerAPI *
MyLocalBrokerIFC::createSubaccount(const std::string &secure_storage_path)
{
	return new MyLocalBrokerIFC(secure_storage_path);
}

void MyLocalBrokerIFC::onLoadApiKey(json::Value keyData)
{
	api_key = keyData["key"].getString();
	symbolExpires = api.now();
}

uint64_t MyLocalBrokerIFC::downloadMinuteData(const std::string_view &asset,
                                              const std::string_view &currency,
                                              const std::string_view &hint_pair,
                                              uint64_t time_from,
                                              uint64_t time_to,
                                              HistData &xdata)
{
    const int CANDLE_INTERVAL = 5 * 60;  // 5 minutes in seconds
    const int FETCH_LIMIT = 500;  // Number of candles to fetch per request
    uint64_t current_time = time_from;
    logDebug("downloadMinuteData", asset, currency, hint_pair, time_from, time_to);

    MinuteData data;

    while (current_time < time_to) {
        uint64_t end_time = std::min(current_time + CANDLE_INTERVAL * FETCH_LIMIT, time_to);

        Value r = publicGET("/market/udf/history", Object{
            {"resolution", "5"},
            {"symbol", std::string(hint_pair)},
            {"from", std::to_string(current_time)},
            {"to", std::to_string(end_time)}
        });

        Value t = r["t"];
        int t_size = t.size();  // Assuming 't' is a JSON array

        for (int i = 0; i < t_size; i++) {
            std::uint64_t tm = t[i].getUIntLong();

            double o = r["o"][i].getNumber();
            double c = r["c"][i].getNumber();
            double h = r["h"][i].getNumber();
            double l = r["l"][i].getNumber();
            double m = std::sqrt(h * l);

            data.push_back(c);
            data.push_back(l);
            data.push_back(m);
            data.push_back(h);
            data.push_back(o);
        }

        std::reverse(data.begin(), data.end());

        if (data.empty()) {
            current_time += CANDLE_INTERVAL * FETCH_LIMIT;
            continue;
        }

		xdata = std::move(data);
        current_time = end_time;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    return  time_to;
}

json::Value MyLocalBrokerIFC::getMarkets() const
{
	updateSymbols();
	std::map<std::string_view, Object> smap;
	for (const auto &x : symbolMap)
	{
		smap[x.second.asset_symbol].set(x.second.currency_symbol, x.first);
	}
	return Object{
		{"Spot", Value(object, smap.begin(), smap.end(), [](const auto &x)
					   { return Value(x.first, x.second); })}};
}

double MyLocalBrokerIFC::getBalance(const std::string_view &symb,
									const std::string_view &)
{
	updateBalances();
	auto iter = balanceMap.find(symb);
	if (iter == balanceMap.end())
		return 0;
	return iter->second;
}

void MyLocalBrokerIFC::onInit() {}

IStockApi::TradesSync
MyLocalBrokerIFC::syncTrades(json::Value lastId, const std::string_view &pair)
{
	const MarketInfoEx &minfo = findSymbol(pair);
	
	int mostID = 0;
	auto findMostID = [&](Value trades)
	{
		for (Value f : trades)
		{
			int t = f["id"].getNumber();
			if (mostID <= t)
			{
				mostID = t;
			}
		}
	};

	if (lastId.getInt() >= 0)
	{

		
		Value trades = privatePOST("/market/trades/list", Object{{"fromId",lastId},{"srcCurrency", minfo.asset_symbol},{"dstCurrency","usdt"}})["trades"];

		if (trades.empty())
		{
			
			return {{}, lastId};
			
		}
		trades = trades.reverse();
		Value ftrades = trades.filter([&](Value r)
			{ return lastId != r["id"]; });
		findMostID(trades);
		if (!trades.empty() && ftrades.empty())
		{
			return {{}, mostID};
		}
		return {
			mapJSON(ftrades,
				[&](Value rw) {
					double size = rw["amount"].getNumber() * (rw["type"].getString() == "buy" ? 1 : -1);
					double price = rw["price"].getNumber();
					double fee = rw["fee"].getNumber();
					double eff_price = price;  
					double eff_size = size;

					std::string feeCurrency = rw["type"].getString() == "buy" ?  minfo.asset_symbol : minfo.currency_symbol; 
					if (feeCurrency == minfo.currency_symbol) {
						eff_price = (price * size + fee) / size; // Adjust if fee affects price
					} else if (feeCurrency == minfo.asset_symbol) {
						eff_size = size - fee;                   // Adjust if fee affects size
					}

					return Trade{
						rw["id"].getUIntLong(),       
						iso8601ToMillis(rw["timestamp"].getString()),  
						size, price,
						eff_size, eff_price
					};
				},
				TradeHistory()),mostID
		};
	}
	else
	{
		Value trades =
			privatePOST("/market/trades/list", Object{{"srcCurrency", minfo.asset_symbol},{"dstCurrency","usdt"}})["trades"];
		findMostID(trades);
		return TradesSync{{}, mostID};
	}
}

bool MyLocalBrokerIFC::reset()
{
	balanceMap.clear();
	orderMap.reset();
	return true;
}


IStockApi::Orders
MyLocalBrokerIFC::getOpenOrders(const std::string_view &pair)
{
	
	const MarketInfoEx &minfo = findSymbol(pair);
	if (!orderMap.has_value())
	{
		Value nb_orders =
			privatePOST("/market/orders/list", Object{{"srcCurrency", minfo.asset_symbol},{"dstCurrency","usdt"},{"status","open"},{"details","2"}})["orders"];

		OrderMap orders;
		orders.reserve(nb_orders.size());
		for (Value row : nb_orders)
		{
			orders.push_back(
				{minfo.asset_symbol+"usdt",
				 Order{std::to_string(row["id"].getInt()), parseOid(row["clientOrderId"]),
					   (row["amount"].getNumber() - row["matchedAmount"].getNumber()) *
						   (row["type"].getString() == "buy" ? 1 : -1),
					   row["price"].getNumber()}});
		}
		orderMap.emplace(std::move(orders));
	}
	Orders res;
	for (const auto &x : *orderMap)
	{
		if (x.first == pair)
			res.push_back(x.second);
	}
	return res;
}

json::Value MyLocalBrokerIFC::placeOrder(const std::string_view &pair,
										 double size, double price,
										 json::Value clientId,
										 json::Value replaceId,
										 double replaceSize)
{
	double createOrderSize = size;
	const MarketInfoEx &minfo = findSymbol(pair);
	if (replaceId.defined())
	{
		
		// cancel the order.
		Value c = privatePOST("/market/orders/update-status",
							  json::Object{
								  {"order", replaceId},
								  {"status", "canceled"}
							  });
		if(size == 0){
			// it was just a cancel order.
			return nullptr;
		}

		
			Value v = privatePOST("/market/orders/status", json::Object{
								  {"id", replaceId}
							  });
			if (v["status"].getString() == "Canceled")
			{
				double remain = v["unmatchedAmount"].getNumber();
				if (remain >= replaceSize )
					createOrderSize = replaceSize;
				else
					return nullptr;
			}else {
				return nullptr;
			}
			
	}
	if (createOrderSize > 0 )
	{
		Value c = privatePOST("/market/orders/add",
							  json::Object{
								{"type", size < 0 ? "sell" : "buy"},
								{"execution","limit"},
								{"srcCurrency", minfo.asset_symbol},
								{"dstCurrency", "usdt"},  
								{"price", price},
								{"amount", std::abs(createOrderSize)},
								// {"clientOrderId", generateOid(clientId)},
							  });
		return c["order"]["id"];
	}

	return nullptr;
}

IBrokerControl::AllWallets MyLocalBrokerIFC::getWallet()
{
	updateBalances();
	Wallet w;
	w.walletId = "spot";
	for (const auto &x : balanceMap)
	{
		w.wallet.push_back({x.first, x.second});
	}
	return {w};
}

IStockApi::Ticker MyLocalBrokerIFC::getTicker(const std::string_view &pair)
{
	std::string upPair(pair);
	std::transform(upPair.begin(), upPair.end(), upPair.begin(), [](unsigned char c)
						   { return std::toupper(c); });
	json::Value res =
		publicGET("/v2/orderbook/" + std::string(upPair), Value());
	// yes, this exchange sends ask and bid incorrectly :)
	return IStockApi::Ticker{
		res["asks"][0][0].getNumber(), res["bids"][0][0].getNumber(),
		res["lastTradePrice"].getNumber(), res["lastUpdate"].getUIntLong()};
}

json::Value MyLocalBrokerIFC::getApiKeyFields() const
{

	Value flds = AbstractBrokerAPI::getApiKeyFields();
	return flds;
}

Value MyLocalBrokerIFC::publicGET(const std::string_view &uri,
								  Value query) const
{
	for (int i = 0; i < 5; i++)
	{
		try
		{
			return processResponse(api.GET(buildUri(uri, query)));
		}
		catch (const HTTPJson::UnknownStatusException &e)
		{
			processError(e);
		}
	}
	throw std::runtime_error("Market overloaded");
}

const std::string &MyLocalBrokerIFC::buildUri(const std::string_view &uri,
											  Value query) const
{
	uriBuffer.clear();
	uriBuffer.append(uri);
	char c = '?';
	for (Value v : query)
	{
		uriBuffer.push_back(c);
		uriBuffer.append(v.getKey());
		uriBuffer.push_back('=');
		simpleServer::urlEncoder([&](char x)
								 { uriBuffer.push_back(x); })(
			v.toString());
		c = '&';
	}
	return uriBuffer;
}

double MyLocalBrokerIFC::getFees(const std::string_view &pair)
{
	return getMarketInfo(pair).fees;
}
bool endsWith(const std::string &str, const std::string &suffix)
{
	if (str.length() < suffix.length())
	{
		return false;
	}
	return str.rfind(suffix) == str.length() - suffix.length();
}
void MyLocalBrokerIFC::updateSymbols() const
{
	auto now = api.now();
	if (symbolExpires <= now)
	{

		Value resp = publicGET("/v2/options", Value());
		Value amountPrecisions = resp["nobitex"]["amountPrecisions"];
		Value pricePrecisions = resp["nobitex"]["pricePrecisions"];
		Value minOrderSizes = resp["nobitex"]["minOrders"];
		SymbolMap::Set::VecT smap;

		amountPrecisions.forEach([&](Value v)
								 {
            std::string symbol = v.getKey();
			std::string symbol_low(symbol);
			std::transform(symbol_low.begin(), symbol_low.end(), symbol_low.begin(), [](unsigned char c)
				{ return std::tolower(c); });
            if (endsWith(symbol_low,"usdt")) {  
                MarketInfoEx nfo;
                nfo.asset_step = v.getNumber();
                nfo.currency_step = pricePrecisions[symbol].getNumber();
                nfo.asset_symbol = symbol_low.substr(0, symbol_low.find("usdt")); 
                nfo.currency_symbol = "usdt";
                nfo.feeScheme = currency;  
                nfo.fees = -1; 
                nfo.invert_price = false;
                nfo.leverage = 0; 
                nfo.min_size = 0;
            	nfo.min_volume = minOrderSizes['usdt'].getNumber();
                nfo.private_chart = false;
                nfo.simulator = false;
                nfo.wallet_id = "spot";
                smap.emplace_back(std::move(symbol_low), std::move(nfo));
            } });

		symbolMap = SymbolMap(std::move(smap));
		symbolExpires = now + std::chrono::hours(1);
	}
}

const MyLocalBrokerIFC::MarketInfoEx &
MyLocalBrokerIFC::findSymbol(const std::string_view &name) const
{
	updateSymbols();
	auto iter = symbolMap.find(name);
	if (iter == symbolMap.end())
		throw std::runtime_error("Unknown symbol");
	return iter->second;
}

void MyLocalBrokerIFC::updateSymbolFees(const std::string_view &name)
{
	updateSymbols();
	auto iter = symbolMap.find(name);
	if (iter == symbolMap.end())
		return;
	// TODO add dynamic feeee
	// via https://api.nobitex.ir/users/profile options object
	iter->second.fees = 0.001;
}

void MyLocalBrokerIFC::updateBalances()
{
	if (balanceMap.empty())
	{
		Value res = privatePOST("/users/wallets/list?type=spot", Object{});
		BalanceMap::Set::VecT b;
		for (Value wallet : res["wallets"])
		{
			std::string_view cur = wallet["currency"].getString();
			// std::string cur_str(cur);
			// std::transform(cur_str.begin(), cur_str.end(), cur_str.begin(), [](unsigned char c)
			// 			   { return std::toupper(c); });
			double balance = wallet["activeBalance"].getNumber();
			b.emplace_back(std::string(cur), balance);
		}
		if (b.empty())
			b.emplace_back(std::string(""), 0.0);
		balanceMap = BalanceMap(std::move(b));
	}
}

json::Value MyLocalBrokerIFC::generateOid(Value clientId) {
    // Generate the next ID
    auto id = nextId++;

    int clientIntId = clientId.getInt();
    int oid = (id << 16) | (clientIntId & 0xFFFF);

    // Return the integer OID
    return json::Value(oid);
}

Value MyLocalBrokerIFC::parseOid(json::Value oid)
{
	if (!oid.defined())
		return json::Value();
	Binary b = oid.getBinary(base64url);
	if (b.empty())
		return json::Value();
	std::size_t c = 0;
	try
	{
		Value v = Value::parseBinary(
			[&]() -> int
			{
				if (c < b.size())
				{
					return b[c++];
				}
				else
					throw std::runtime_error("invalid oid");
			},
			base64url);
		unsigned int id = v[0].getUInt();
		if (id > nextId)
			nextId = id;
		return v[1];
	}
	catch (...)
	{
		return json::Value();
	}
}

json::Value MyLocalBrokerIFC::processResponse(json::Value v) const
{
	if (v["status"].getString() == "ok")
		return v;
	std::ostringstream buff;
	buff << v["code"].getString() << " " << v["message"].getString();
	throw std::runtime_error(buff.str());
}

void MyLocalBrokerIFC::processError(
	const HTTPJson::UnknownStatusException &e) const
{
	std::ostringstream buff;
	buff << e.getStatusCode() << " " << e.getStatusMessage();
	try
	{
		auto s = e.response.getBody();
		json::Value error = json::Value::parse(s);
		if (e.getStatusCode() == 429)
			return;
		buff << " - " << error["code"].getString() << " "
			 << error["message"].getString();
	}
	catch (...)
	{
	}
	throw std::runtime_error(buff.str());
}

bool MyLocalBrokerIFC::hasKey() const { return !(api_key.empty()); }

Value MyLocalBrokerIFC::privateGET(const std::string_view &uri,
								   Value query) const
{
	for (int i = 0; i < 5; i++)
	{
		try
		{
			std::string fulluri = buildUri(uri, query);
			return processResponse(
				api.GET(fulluri, std::move(signRequest())));
		}
		catch (const HTTPJson::UnknownStatusException &e)
		{
			processError(e);
		}
	}
	throw std::runtime_error("Market overloaded");
}

Value MyLocalBrokerIFC::privatePOST(const std::string_view &uri,
									Value args) const
{
	for (int i = 0; i < 5; i++)
	{
		try
		{

			return processResponse(api.POST(uri, args, std::move(signRequest())));
		}
		catch (const HTTPJson::UnknownStatusException &e)
		{
			processError(e);
		}
	}
	throw std::runtime_error("Market overloaded");
}

Value MyLocalBrokerIFC::privateDELETE(const std::string_view &uri,
									  Value query) const
{
	for (int i = 0; i < 5; i++)
	{
		try
		{
			std::string fulluri = buildUri(uri, query);
			return processResponse(api.DELETE(
				fulluri, Value(), std::move(signRequest())));
		}
		catch (const HTTPJson::UnknownStatusException &e)
		{
			processError(e);
		}
	}
	throw std::runtime_error("Market overloaded");
}

Value MyLocalBrokerIFC::signRequest() const
{
	Value headers = Object({{"Authorization",
							 "Token " + api_key},
							{"Amin", "mmb"}});

	return headers;
}

unsigned long MyLocalBrokerIFC::iso8601ToMillis(const std::string &timeStr)
{
	std::istringstream ss(timeStr);
	std::tm tm = {};
	ss >> std::get_time(&tm, "%Y-%m-%dT%H:%M:%S");

	if (ss.fail())
	{
		throw std::invalid_argument("Invalid ISO 8601 format: " + timeStr);
	}

	// Handle fractional seconds (optional)
	long long milliseconds = 0;
	char dot;
	if (ss >> dot)
	{
		int millisPart;
		ss >> millisPart;
		milliseconds = millisPart;
	}

	// Handle timezone offset (optional)
	int offsetHours = 0, offsetMinutes = 0;
	char sign;
	if (ss >> sign)
	{
		ss >> offsetHours >> std::ws; // Read hours and discard colon
		ss >> offsetMinutes;
		milliseconds += (sign == '+' ? -1 : 1) * (offsetHours * 3600000 + offsetMinutes * 60000);
	}

	// Convert to milliseconds since epoch
	std::chrono::system_clock::time_point tp = std::chrono::system_clock::from_time_t(std::mktime(&tm));
	auto duration = tp.time_since_epoch();
	return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() + milliseconds;
}