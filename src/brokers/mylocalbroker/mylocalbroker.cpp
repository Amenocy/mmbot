#include "mylocalbroker.h"

#include <sstream>

#include <random>

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

static std::string_view
	licence(R "mit(Copyright (c) 2019 Ondřej Novák

			Permission is hereby granted,
			free of charge,
			to any person obtaining a copy of this software and associated
				documentation files(the "Software"),
			to deal in the Software without restriction,
			including without limitation the rights to use, copy, modify, merge,
			publish, distribute, sublicense,
			and / or sell copies of the Software,
			and to permit persons to whom the Software is furnished to do so,
			subject to the following conditions
			:

			The above copyright notice and this permission notice shall be
					included in all copies or
				substantial portions of the Software.

				THE SOFTWARE IS PROVIDED "AS IS",
			WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
			INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
			FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT
				SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
			DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
			TORT OR OTHERWISE, ARISING FROM,
			OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
				DEALINGS IN THE SOFTWARE.) mit ");

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
		  "https://testnetapi.nobitex.ir") {}

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
					  false};
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
	updateSymbols();
	auto iter = symbolMap.find(hint_pair);
	if (iter == symbolMap.end())
	{
		iter = std::find_if(symbolMap.begin(), symbolMap.end(),
							[&](const auto &x)
							{
								return x.second.currency_symbol == currency &&
									   x.second.asset_symbol == asset;
							});
	}
	MinuteData data;
	time_from /= 1000;
	time_to /= 1000;
	if (iter != symbolMap.end())
	{
		Value r =
			publicGET("/api/v1/market/candles", Object{
													{"type", "5min"},
													{"symbol", iter->first},
													{"startAt", time_from},
													{"endAt", time_to},
												});
		std::uint64_t minTime = time_to;
		for (Value rw : r)
		{
			std::uint64_t tm = rw[0].getUIntLong();
			if (tm >= time_from && tm < time_to)
			{
				double o = rw[1].getNumber();
				double c = rw[2].getNumber();
				double h = rw[3].getNumber();
				double l = rw[4].getNumber();
				double m = std::sqrt(h * l);
				while (tm + 5 * 60 < minTime)
				{
					minTime -= 60;
					data.push_back(c);
				}
				data.push_back(c);
				data.push_back(l);
				data.push_back(m);
				data.push_back(h);
				data.push_back(o);
				minTime = tm;
			}
		}
		std::reverse(data.begin(), data.end());
		if (data.empty())
		{
			return 0;
		}
		else
		{
			xdata = std::move(data);
			return minTime * 1000;
		}
	}
	else
	{
		return 0;
	}

	return 0;
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

	Array mostIDS;
	std::uint64_t mostTime = 0;
	auto findMostTime = [&](Value fills)
	{
		for (Value f : fills)
		{
			std::uint64_t t = f["createdAt"].getUIntLong();
			if (mostTime <= t)
			{
				if (mostTime < t)
					mostIDS.clear();
				mostIDS.push_back(f["tradeId"]);
				mostTime = t;
			}
		}
	};

	if (lastId[0].getUIntLong() > 0)
	{

		bool timeOverflow = false;
		std::uint64_t startAt = lastId[0].getUIntLong();
		std::uint64_t endAt =
			std::chrono::duration_cast<std::chrono::milliseconds>(
				api.now().time_since_epoch())
				.count();
		if (endAt - startAt > 6 * 24 * 60 * 60 * 1000)
		{
			endAt = startAt + 6 * 24 * 60 * 60 * 1000;
			timeOverflow = true;
		}

		Value fills =
			privateGET("/api/v1/fills", Object{{"symbol", pair},
											   {"pageSize", 500},
											   {"startAt", startAt},
											   {"endAt", endAt}})["items"];
		if (fills.empty())
		{
			if (timeOverflow)
			{
				return {{}, {(startAt + endAt) >> 1, {}}};
			}
			else
			{
				return {{}, lastId};
			}
		}
		fills = fills.reverse();
		findMostTime(fills);
		Value ffils = fills.filter([&](Value r)
								   { return lastId[1].indexOf(r["tradeId"]) == Value::npos; });
		if (!fills.empty() && ffils.empty())
		{
			return {{}, {mostTime + 1, mostIDS}};
		}
		return {
			mapJSON(
				ffils,
				[&](Value rw)
				{
					double size = rw["size"].getNumber() *
								  (rw["side"].getString() == "buy" ? 1 : -1);
					double price = rw["price"].getNumber();
					double fee = rw["fee"].getNumber();
					double eff_price = price;
					double eff_size = size;
					std::string_view feeCurrency =
						rw["feeCurrency"].getString();
					if (feeCurrency == minfo.currency_symbol)
					{
						eff_price = (price * size + fee) / size;
					}
					else if (feeCurrency == minfo.asset_symbol)
					{
						eff_size = size - fee;
					}
					return Trade{rw["tradeId"], rw["createdAt"].getUIntLong(),
								 size, price,
								 eff_size, eff_price};
				},
				TradeHistory()),
			{mostTime, mostIDS}};
	}
	else
	{
		Value fills =
			privateGET("/api/v1/fills", Object{{"symbol", pair}})["items"];
		findMostTime(fills);
		if (mostTime == 0)
			mostTime = std::chrono::duration_cast<std::chrono::milliseconds>(
						   api.now().time_since_epoch())
						   .count();
		return TradesSync{{}, {mostTime, mostIDS}};
	}
}

bool MyLocalBrokerIFC::reset()
{
	balanceMap.clear();
	orderMap.reset();
	return true;
}

void MyLocalBrokerIFC::updateOrders()
{
	if (!orderMap.has_value())
	{
		Value res = privateGET("/api/v1/orders", Object{
													 {"status", "active"},
													 {"pageSize", 500},
												 })["items"];

		OrderMap orders;
		orders.reserve(res.size());
		for (Value row : res)
		{
			orders.push_back(
				{row["symbol"].getString(),
				 Order{row["id"].getString(), parseOid(row["clientOid"]),
					   (row["size"].getNumber() - row["dealSize"].getNumber()) *
						   (row["side"].getString() == "buy" ? 1 : -1),
					   row["price"].getNumber()}});
		}
		orderMap.emplace(std::move(orders));
	}
}

IStockApi::Orders
MyLocalBrokerIFC::getOpenOrders(const std::string_view &pair)
{
	updateOrders();
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
	if (replaceId.defined())
	{
		std::string orderURI("/api/v1/orders/");
		orderURI.append(replaceId.getString());
		privateDELETE(orderURI, Value());

		do
		{
			Value v = privateGET(orderURI, Value());
			if (v["isActive"].getBool() == false)
			{
				double remain =
					v["size"].getNumber() - v["dealSize"].getNumber();
				if (remain > replaceSize * 0.95)
					break;
				else
					return nullptr;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		} while (true);
	}
	if (size)
	{
		Value c = privatePOST("/api/v1/orders",
							  json::Object{
								  {"clientOid", generateOid(clientId)},
								  {"side", size < 0 ? "sell" : "buy"},
								  {"symbol", pair},
								  {"type", "limit"},
								  {"stp", "DC"},
								  {"price", price},
								  {"size", std::abs(size)},
								  {"postOnly", true},
							  });
		return c["orderId"];
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
	json::Value res =
		publicGET("/v2/orderbook/" + std::string(pair), Value());
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

		amountPrecisions.forEach([&](Value v){
            std::string symbol = v.getKey();

            if (symbol.find('USDT') != std::string_view::npos) {  
                MarketInfoEx nfo;
                nfo.asset_step = 1.0 / pow(10, v.getNumber());
                nfo.currency_step = 1.0 / pow(10, pricePrecisions[symbol].getNumber());
                nfo.asset_symbol = symbol.substr(0, symbol.find("USDT")); 
                nfo.currency_symbol = "USDT";
                nfo.feeScheme = currency;  
                nfo.fees = -1; 
                nfo.invert_price = false;
                nfo.leverage = 0; 
                nfo.min_size = minOrderSizes['usdt'].getNumber();
            	nfo.min_volume = 0;
                nfo.private_chart = false;
                nfo.simulator = false;
                nfo.wallet_id = "spot";
                smap.emplace_back(std::move(symbol), std::move(nfo));
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
        Value res = privateGET("/users/wallets/balance", Object{});
        BalanceMap::Set::VecT b; 
        for (Value wallet : res["wallets"]) 
        {
            std::string_view cur = wallet["currency"].getString();
            double balance = wallet["activeBalance"].getNumber();
            b.emplace_back(std::string(cur), balance); 
        }
		if (b.empty())
			b.emplace_back(std::string(""), 0.0);
        balanceMap = BalanceMap(std::move(b));
    }
}

json::Value MyLocalBrokerIFC::generateOid(Value clientId)
{
	auto id = nextId++;
	Value ctx = {id, clientId.stripKey()};
	std::basic_string<unsigned char> oidBuff;
	ctx.serializeBinary([&](char c)
						{ oidBuff.push_back(c); });
	return json::Value(json::BinaryView(oidBuff), base64url);
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

// Doooooooooooooooooone part

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
				api.GET(fulluri, signRequest("GET", fulluri, Value())));
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
			return processResponse(
				api.POST(uri, args, signRequest("POST", uri, args)));
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
				fulluri, Value(), signRequest("DELETE", fulluri, Value())));
		}
		catch (const HTTPJson::UnknownStatusException &e)
		{
			processError(e);
		}
	}
	throw std::runtime_error("Market overloaded");
}

Value MyLocalBrokerIFC::signRequest(const std::string_view &method,
									const std::string_view &function,
									json::Value args) const
{

	Value s = Object({"Authorization", "Token " + api_key});
	logDebug("SIGN: $1", s.toString().str());
	return s;
}