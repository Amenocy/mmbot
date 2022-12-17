#include "coinbase_advanced.h"

#include <imtjson/object.h>
#include <imtjson/parser.h>
#include <simpleServer/urlencode.h>

#include <iostream>
#include <iomanip>
#include <openssl/hmac.h>
#include <sstream>
using json::Object;

int main(int argc, char **argv) {
    using namespace json;

    if (argc < 2) {
        std::cerr << "Required one argument" << std::endl;
        return 1;
    }

    CoinbaseAdv ifc(argv[1]);
    ifc.dispatch();
}



CoinbaseAdv::CoinbaseAdv(const std::string &path)
:AbstractBrokerAPI(path,{
    Object({
        {"name","key"},
        {"label","API Key"},
        {"type", "string"}
    }),
    Object({
        {"name","secret"},
        {"label","API Secret"},
        {"type", "string"}
    })
})
,httpc(simpleServer::HttpClient("MMBot Mozilla/5.0 (compatible; MMBot/2.0; +https://github.com/ondra-novak/mmbot.git)", simpleServer::newHttpsProvider(), nullptr, nullptr),"https://api.coinbase.com")
,ws(*this, httpc.getClient(), "https://advanced-trade-ws.coinbase.com")
{
}

std::string CoinbaseAdv::calculate_signature(std::uint64_t timestamp,
                        std::string_view method, std::string_view reqpath,
                        std::string_view body) const {
    std::ostringstream buff;
    buff << timestamp << method << reqpath << body;
    std::string s (std::move(buff.str()));
    unsigned char digest[50];
    unsigned int digest_size = sizeof(digest);
    HMAC(EVP_sha256(),api_secret.data(), api_secret.size(), 
            reinterpret_cast<const unsigned char *>(s.data()), s.size(), digest, &digest_size);
    buff.str(std::string());
    for (unsigned int i = 0; i < digest_size; i++) {
        buff << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(digest[i]); 
    }
    return buff.str();
}

void CoinbaseAdv::processError(HTTPJson::UnknownStatusException &e) const{
    json::Value v;
    try {
        auto s = e.response.getBody();
        v = json::Value::parse(s);
    } catch (...) {
        throw e;
    }
    throw std::runtime_error(std::string(e.what()).append(" ").append(v.toString()));
    
}

json::Value CoinbaseAdv::GET(std::string_view uri, json::Value query) const {
 try {
    std::ostringstream request;
    request << uri;
    if (!query.empty()) {
        char c = '?';
        for (json::Value v: query) { 
            request << c << v.getKey() << "=" << simpleServer::urlEncode(v.toString());
            c = '&';
        }
    }
    return httpc.GET(request.str(), headers("GET", uri, ""));
 } catch (HTTPJson::UnknownStatusException &e) {
     processError(e);
 }
}


json::Value CoinbaseAdv::POST(std::string_view uri, json::Value body) {
 try {
    auto b = body.stringify();
    return httpc.POST(uri, b, headers("POST", uri, b));
 } catch (HTTPJson::UnknownStatusException &e) {
     processError(e);
 }

}

json::Value CoinbaseAdv::headers(std::string_view method,std::string_view reqpath, std::string_view body) const {
    auto tm = std::chrono::duration_cast<std::chrono::seconds>(httpc.now().time_since_epoch()).count();
    json::Value v = json::Object {
        {"Content-Type",method == "GET"?json::Value():json::Value("application/json")},
        {"CB-ACCESS-KEY",api_key},
        {"CB-ACCESS-SIGN",calculate_signature(tm, method, reqpath, body)},
        {"CB-ACCESS-TIMESTAMP", tm},
        {"CB-VERSION","2022-12-16"}
    };
    if (debug_mode) {
        std::cerr << v.toString() << std::endl;
    }
    return v;
}



IBrokerControl::BrokerInfo CoinbaseAdv::getBrokerInfo() {
    return {
        !api_key.empty() && !api_secret.empty(),
        "Coinbase",
        "Coinbase",
        "https://coinbase.com/join/novk_3k?src=referral-link",
        "1.0.0",
        "MIT Licence",
        "iVBORw0KGgoAAAANSUhEUgAAAIAAAACACAYAAADDPmHLAAASAklEQVR42u3d"
        "ebSe0/UH8M+5uTeRK5KQGDJKxBBCm6AUEXMtQ0lugppasypRpUpTVfyo1tT6"
        "mamhSo33DYqa/aJBa/YTQ4LMSCREJDLc6fn98Vz9tSXDve953vvem/e71rus"
        "Fevu5zxnf5999t5nn30ooYQSSiihhBJKKKGEElYthDb3RlVJB6yDvtgA/Rt/"
        "fbEuuqITKlCJBItRg4X4DLMwDVMaf5MxA3PkQk2JAMWj7CBYTWJDDMNQbIGe"
        "WAPlkZ5UiwX4AK/jGYzXYAqWui8kJQIUVvHdGxW+J3Zu/NLLCzyKWkzC03hU"
        "g/HuC5+VCJCd0jvh2/h+o+K7o6xIRleP2XgYt+IlubC4RIAYGJn0ljgch2Cz"
        "IlL68sjwOm7H7XJhdokATcVBSVBrIH6Iw7BmK1yuEszBzbheLkwuEWBFGJ4E"
        "ZfrjTByILm3E2Z6L23CpXJhZIsDXr/FrYzROavzi2yI+xu8E16ouDoex5Qlw"
        "QFKh3ghcgAFtMjfx1aXhbZyhwiPuCnWrLgGqkgH4LYajnVULtbgDY+TCB6sW"
        "AUYk5YIDcBnWs2pjmuBkiQflQkOhH174kGpk0lVwNf5YUj5YX+JuXGxU0qlt"
        "W4CqZDPcKE3olPBV3+ApHFfIkLEwBBieBO3sIXETepV0vVxMxmFy4fm2QYCj"
        "kuAzR+By6QZNCSvGpzhOImdsthtN2foAByXlPnMqrikpv0lYC7cKjnZoUtY6"
        "CXBwUqbWmbgQHYpllQ0oS9Ktw4rGX3njv4WkcSUuDlTiCouNdniSmaXORnAa"
        "5p2Jc1okvm9UZLfVGdqLTXvTe13W60b3rlR2pEN7yhrpX9/A0qUsXsLH85g1"
        "lxmzeWsG4z5k4ZIWDZprcZpKV7ktfpgYfw/9yCSY72ScXWjlr1fBNv3Yd1u+"
        "sTHr96Dz6qzWnrImKq++gSU1fL6QKR/y2kQefIGXpjOnsLm7ClxkkS9wU3Fb"
        "gP2ToJ3v47pCmv2DB7H/ULbajPXXoyKj0pCltSkZXprA2PHkJhWUCItwqFy4"
        "r3gJUJXshrGFcPgGVHLIUEbuxsC+qUkvJBbX8NZk7n6cP/+dmUsK8thPBHup"
        "Di8WHwGqkk3wBHpnOQPdO/DzvRm+C/16Nt20x0ZDA+/NpPpJfv0IC7NfHiZh"
        "D7kwvXgIUJV0kZZDbZ/VW7dvx2nDOGo4G/RqecV/HRHemc4NOX7/XObRxMMa"
        "jHJf/mVn+U/jyKSdxBU4Iau33b0v5xzBNoOyW99j+gnPvs5ZN/F8dsVgCS5Q"
        "51ceyC8yCBG+/lHS+rf4q3Dgov3Sr75b59aVyZn9Kdfcw7mPZegUBvurDk+0"
        "HAGqkvXxrAzy+1t04ZLj2PVblLfSSoGaOh4ez+k38d4XGfkDwVDVYU7hCXBQ"
        "Uq7WrTg49lvtP4ALf8Sm/bQJvDaJU6/k6WzKPq7G6ObWEjQ/FVxnXxwQ+22O"
        "GsK1Z7Qd5cPgjbl5DAcMzET8UdixsBZgZLKWxHPYJOab/GQHfnksa7bRbaOP"
        "5/Hzq7jp1eiiX8LOcuGL7C3AyCRInICNY77BKTvwq+PbrvJhnTX5zUkcNTi6"
        "6C1xeGGWgAa98WMRk0hHDeHsY+myujaPtbvy6xMZGfXzUYYzjUy6ZU+A4KdY"
        "O9bI9xvA+Se07S//P7HuWlx8MjvGrYhcX+KkbAmQlnEfGmvEm3fmwhPo0c0q"
        "h/49uOwk+nWMKvZYI5MeWVqAYxBHXSGN8zfrb5XFVpty0RFRRfaSNO0DXXkC"
        "jEzWwZGxRnrhvuy2jVUaAfvtxBk7RxV7ghHJGvEJkDhE2nolb+zSm2NGtN4M"
        "X0x0qGD09xi8VrzVRTA8LgFGJB0b1/68Pf/yMs47ku5dSsr/p91emwuOiGpY"
        "fqAqqVgpfaykyC0RJXo9dUe23bxlJrq2jkVL+HQB8xZQW5v+e/sKuq7BWp2p"
        "7NAyO467bM0xW/KHV6KI216apJsQhwBpkiHvaVmzPUePKOwEL17KO1N5+S2e"
        "e5PH32PmAjR81Rb26cweA9h+EFsNYpN+dCxQpVHHDvxwJLe8Rl3+pZ8dpf0V"
        "JqyMuViR+e8smIA++Y7qov057fDCFHN8tpBxL3P9X3hmGgubOKlrlLFTf47b"
        "l2Fb0qUAp/bqG/jF1fz26Sji3hJsqToszc8HCLZHj3xH068jI3bNXvk1dTzy"
        "PN/7JcMv5+EpTVc+LGjgwffZ7/cceg6P/yNdQrJEuzIO2ZOucSzkhpIVL9tl"
        "Kwj9grQjV95DOnj7tJQrS3z0CWdfy76X8uh0cZLVgYemsNfFnHtDWuiRJTbt"
        "z4Fx9gra4zv5RgHtsUuM0Rywe7Zf/5tTOPL81HzWZ1CPV59wwRMc82smTsvu"
        "PSrKUysQqaZwNyOS8nwI0Bd572KPGpjt/v6rEzn4/MavPmM8OIVDLkiLPLLC"
        "lgPZoWcUUYOtIHO7fAI02E6EAx4jhqanc7LAW1P4wUW8UcCWS698wjGXMjEj"
        "wnXqyCHDoojqItiq+QQI8h7G2hVsPSibiZr1Caf+d2GV/yVensvPrmROBs8O"
        "gW9/k8o4R3eHNY8Ao5LyGMmfb/WhX4/4k1RTx+V3FsbsLwsPvM9Vd1NXH1/2"
        "hn3YPM6221YOXPbp4rLlmP9ujT5A85GwzzZppi02nn6Ri5/W4rjgMf4Wv8RL"
        "50r2HBJF1EZql31Ub3lGphc650uAwZvEn5zPFnL5Pdl4+01FXcIV97JgUXzZ"
        "O3zDVzOWTUc3y9nEWx4B+svzsMcaHenfM/7EPPMKf52qaDB2Es//b3y5/XuL"
        "ccC+o7BsS162HAdw/XxTKcN60jlyCnVxDTc8qLj6iQZufCj1S2JinTX5Zv7b"
        "xO2wftMJkMg7ct+sd/zwb9I0xk1RdHhiIu/NiCuzYweGxDlr3ac5S0DffJ/a"
        "e700vx01/HozzdMXGz6t59W348psX0G/daOI6tUcAuQdvK0Xudizto5n31S0"
        "ePHtuCFhwLrdo4hapi6XlyfOa/UJSdqQKer6vzTdzy9WPPlOejQ8VtgbAh/H"
        "STR1bw4BVs+XvZ0q407wJ58z4/PiJcCET9n5fHGbQ4QoDm/n5hBgtXzZ2z6y"
        "Azh/YWNcXMy3BsVRWGx0bI4P0D7veYg8EUuWKiEyykpTUCLAspDXFakJksip"
        "2tU6lBQWG8vzAfLqfJck1ES+Zbfz6q3AZsXOUcTxKRY3hwBf5GsBFkW+O7N7"
        "F3quwYcLilP3Q7pz8dHxyt4DnniR8/JvNLWgOQSYlxcBQvxiiY4d+M6AtHa+"
        "GLHbZuy8ddzs51tx0t5zm+MDfJTvU2fNjTvBFeXsMEjRYstN4io/wexPooia"
        "1RwC5L21MXN22kEz6iQPYvUi9AO6lTM4chOomlqmzooiamZzCDA1b/M1M92+"
        "jYmB/Rjat/gIsMcmDIjcJXnJUl6P01puRosQ4G8fsCByg8TKDhy3j2K62YOE"
        "I/emfeQzj3M+SyuQI8Ql05pDgCn55gLmL2HqR/Hne+et2a1P8eh/xMZs9434"
        "cifPlF5Gnx8WYXpzCPAB8t56eW1i/IlZqzOnjCqOlEB5YPQo1qiML/u5N6Lk"
        "PT6VXlrdRAKU+WR5zFnZQPahF+KXSsEe3+bHQ1ueAGP2YOiQ+HIXLObRONXG"
        "76ld9oe8bALcG2qRd6nji9OYNiv+BHWo4NTD2KVnyyl/3/6ceCAVGbS6eX8G"
        "E+ZEEfWyvyz77sEVGZhx+T59dm1axpUFeq/N705m4xboMfjNNbl4dFq4Gd2n"
        "TPj768071v41eGZ5/3NFBHg+X0cQ7n82vUghE0VsxK2ns3EBr13eois3n87A"
        "9bOR/8US7hofRdTneLn5BKg3VXpHTV64803eyfBI9baDuOsXDOuRvfK/05c7"
        "z2LIJtk949WJ/E+cCuPXMaf5BGinBlEOYFU/SUOGsfvgjbntbEZvl01BThlO"
        "34lbzsq2uWVtPXc8JtZLPCUX6ppPgFxI8GiMaPTP47PJCfwr+qzDRSeTG82O"
        "PcRJFiXs1psHfpL2NM66re3EqdwTp1NYDR5bGWIvHw2eRd7XH72/iPufjl8k"
        "8p9YrT3DdyZ3AXccz0696NCMr2m1kF5Wdc+J3P1f7DM0m0Ou/7biNnDX48yN"
        "4y9N0WCFVFq5qalKbpD2Cc4La3fg+YsZkHGvoP+Mp998P73tc9wEHpvK5198"
        "/Ux0rWTP/gzbIu1pMGgAq69WuLG+9i7bjqEmjvd/oVwYE4sAw/CUCEcVx+zK"
        "OT/MJnZeUWhVW5ee4p07P60w/rJiqX37tA1c967pgdaK8vgFrSvCkhp+chnX"
        "xrkTdAm2lQsrzOOs7PbFS3hDhIYRl4xj32Fst0VhJziE1IR365L+ig3PvBJN"
        "+fAPvLOyzu2KkQuLpHcD5r2C19Rzzs1pq9YSUnw0l1/cHM/Y4Y9yoSYeAVKx"
        "tyNKfcpj07jp/mxaq7Q21NRx9T28FK96arog15TwduUwNnyEP8Ya5en3p2Zv"
        "VUYivVjy/Ceiir1OdZgfnwCpy3i9PItF//nyDZx2HZOmr7oEeH1SeqtozNUE"
        "tzblD5pGgArv4s5oYc88xlyT3qe3qmHGx/z0quhXyt4oFz7IjgB3hgQXxfIF"
        "oHoSv7qu8eDnKoK5n3HGlTw5M6rYmbiiqX/U9Gh84DnzBZXYSaSM9Usf0jAv"
        "LavKOtvW0pi3gLOujX62oQFny4WnmvqHTS84GhsSwZV4P+YbXDyO8//Qti3B"
        "J/M562qufSG+OyHRrECy+V9wVXIA7hCjkdm/4IRtOPf49IbNtoQP5nDmldw2"
        "IbroGuwjF5oVS+RTcngfxsZ+m2te4MSLeHdG21H+m5M55sJMlA+3Spq/ZZ/f"
        "Gl6VbIBnsV7st9pqLS45gR0Hx+80VijU1ac3jZx2PW9nk/mcLLGDsWFWyxAg"
        "JcEhuAXR3bd2ZVxWxfe/S9dOrUv5c+dzQ44xD8nqEMsSVMmFv+YjJMa3dbeI"
        "GcJ/RX0Dp9zL4efy4lutI3VcW8f41zjwbMY8mJnyE1wheDRfQXE2PauSNfEI"
        "MrsMtnM5p+7GEfvRd93Cb9euMA5LmPwBN47l0meozbaZ5ZOC/VWHL4qDADAy"
        "2VziMREaTC4PvSo5a3/23jEtAWtpIjQkTP2Q+57mwoeZm30jq8nYXS5E6RwQ"
        "d/qqkr1wjzx7DK4MNl2Dw3di+C7pqdz2Bb7tc2ltuo9x75Pc/re05K0AmCf4"
        "rurwbCyBcQlwQBLUOxZXZuEULsuLOWYwe2/PkE1Tq5BV1FBXz/TZ6UGXB55t"
        "DOsKd0p5MY6UC3fFFBrfgB6WlFnkTJwrwn2DTXmR3h3YYSP23obNN0rJ0Kky"
        "PUbW1KUiSdIyrQWLmD6LN97lwX/w98l8VFPw0+k1gp+oc437Q1LcBIDhSbky"
        "5+EMLXGIt3GK+nVmuz5s3Jve69KzO2t2pmPHdM+hvJGedXUsrUmbWs37PM3a"
        "fTCbiTMZP4MPF2Y6WysMLHCmCpe7K9Rn8eFkgxFJueA8nF5IS7AiYgS0C+l/"
        "v2Rmw5e/JG1uVUSowRiru8yfQiZGJ9vXrUrKcSbOLphP0HawBD9T4eosvvzC"
        "EABGJWUaHI9LUFnS60phPn6kqzvcFDJ1Nwpj8EYmQWJf3IB1S/pdLqZL/EA7"
        "49wbMvc1C7viVSWDcbMI5wvaKMbjaLkwqVAPLKyHnguvSeyOP4nR/qjtoBbX"
        "CPYppPJbLrAZlVRocDh+aznXmawi+BCnCHKqQ8E/ipYNekYkAwWXYC+r3t0F"
        "9agWnKE6TG2pQbR81Dsi6SD4Hs4T4aq6VoBEWk/5cw3ud1+obcnBFE/aY0TS"
        "Q3AqjpPvncXFi09xpeAK1WFuMQyouPJew5OgzCYYgxHo1EYUP19aOPMbiSnG"
        "hqJpdBuKcrr2T8q0szlOwkGt1CIk0mN0t+FaFd5xV0iKbZChqKdwdBLM1F9w"
        "ZCMRNiz6MafbCm/jz7hVLsws5sGGVvM9VSWdMQxHYBd0LaLIoUF6XO5xwS14"
        "XnVoFUdcWg8B/p0MPbAr9mwkRZ8WIEO9tKX+ODwieEZ1mN3aprJ1EuD/I4cy"
        "ZSolNsXO2AGbSc8pVIp3aqkeC6Xd0t6QpmzHCSZiseriW9tXDQJ8HUYmlRLr"
        "oT8GYIPGX1+s3bh0dPLVm1GXNip5nrS75jRpAebkxrh9KmbLhcVKKKGEEkoo"
        "oYQSSiihhBJaLf4PuqgOpn080vYAAAAASUVORK5CYII=",
        false,
        true,
        false,
        false
    };
}


std::vector<std::string> CoinbaseAdv::getAllPairs() {
    json::Value res = GET("/api/v3/brokerage/products",nullptr);
    std::vector<std::string> out;
    for (json::Value v: res["products"]) {
        if (v["product_type"].getString() == "SPOT" && !v["is_disabled"].getBool()) {
            out.push_back(v["product_id"].getString());
        }
    }
    return out;
}

json::Value CoinbaseAdv::getMarkets() const {
    std::map<json::String, json::Object> ids; 
    json::Value res = GET("/api/v3/brokerage/products",nullptr);
    for (json::Value v: res["products"]) {
        if (v["product_type"].getString() == "SPOT" && !v["is_disabled"].getBool()) {
            auto qn = v["quote_name"].toString();
            ids[qn].set(v["base_name"].getString(), v["product_id"]);
        }
    }    
    return json::Value(json::object, ids.begin(), ids.end(), [](const auto &v){
        return json::Value(v.first, v.second);
    });
    
}



AbstractBrokerAPI* CoinbaseAdv::createSubaccount(
        const std::string &secure_storage_path) {
    return new CoinbaseAdv(secure_storage_path);
}

bool CoinbaseAdv::areMinuteDataAvailable(const std::string_view &asset,
        const std::string_view &currency) {
}

json::Value CoinbaseAdv::placeOrder(const std::string_view &pair, double size,
        double price, json::Value clientId, json::Value replaceId,
        double replaceSize) {
    throw std::runtime_error("Not yet supported");
}


void CoinbaseAdv::reject_all_tickers() {
    for (auto &x: _orderBooks) {
        x.second._buy.clear();
        x.second._sell.clear();
        if (x.second._wait.has_value()) {
            x.second._wait->set_value();
            x.second._wait.reset();
        }        
    }
}

IStockApi::Ticker CoinbaseAdv::getTicker(const std::string_view &pair) {
    std::string p(pair);
    std::unique_lock _(_ws_mx);
    if (_orderBooks.empty()) {
        ws.regHandler([this](WsInstance::EventType event, json::Value data){
            std::unique_lock _(_ws_mx);
            switch (event) {
                case WsInstance::EventType::data: {
                    if (data["channel"].getString() == "l2_data") {
                        for (json::Value event: data["events"]) {
                            std::string_view type = event["type"].getString();
                            if (type == "update" || type == "snapshot") {
                                bool snapshot = event["type"].getString() == "snapshot";
                                std::string_view product = event["product_id"].getString();
                                auto iter = _orderBooks.find(product);
                                
                                if (iter == _orderBooks.end() || iter->second._expires < std::chrono::system_clock::now()) {
                                    ws.send(ws_subscribe(true, {product}, "level2"));
                                    if (iter != _orderBooks.end()) _orderBooks.erase(iter);
                                    continue;                                
                                }
                                auto &o = iter->second;
                                for (json::Value v: event["updates"]) {
                                    auto &side = v["side"].getString()=="bid"?o._buy:o._sell;
                                    double q = v["new_quantity"].getNumber();
                                    double p = v["price_level"].getNumber();
                                    if (q == 0) side.erase(p);
                                    else side[p] = q;
                                }
                                if (o._wait.has_value()) {
                                    o._wait->set_value();
                                    o._wait.reset();
                                }                        
                            } else {
                                std::cerr << data.toString() << std::endl;
                                reject_all_tickers();
                            }
                        }
                    } else {
                        std::cerr << data.toString() << std::endl;
                    }
                }break;
                case WsInstance::EventType::connect: {
                    std::vector<std::string_view> products;                    
                    std::transform(_orderBooks.begin(), _orderBooks.end(), std::back_inserter(products),[](auto &x) -> std::string_view{
                        return x.first;
                    });
                    if (!products.empty()) {
                       ws.send(ws_subscribe(false,products, "level2"));
                    }
                        
                }break;
                case WsInstance::EventType::disconnect: 
                case WsInstance::EventType::exception: {
                    std::unique_lock _(_ws_mx);
                    reject_all_tickers();
                }
                break;                
            }
            return !_orderBooks.empty();
        });
    }
    auto newexp = std::chrono::system_clock::now()+std::chrono::minutes(2);
    auto iter = _orderBooks.find(p);
    if (iter == _orderBooks.end() || iter->second._buy.empty() || iter->second._sell.empty()) {
        auto &o = _orderBooks[p];
        o._wait.emplace();
        o._expires = newexp;
        auto f = o._wait->get_future();
        bool ok;
        _.unlock();
        ws.send(ws_subscribe(false,{p}, "level2"));
        ok =  f.wait_for(std::chrono::seconds(5)) ==std::future_status::ready; 
        if (!ok) {
            ws.send(ws_subscribe(true,{p}, "level2"));
        }
        _.lock();
        if (!ok) {
            _orderBooks.erase(p);            
            throw std::runtime_error("Failed to get ticker");
        }
    }
    iter = _orderBooks.find(p);
    if (iter == _orderBooks.end() || iter->second._buy.empty() || iter->second._sell.empty()) {
        throw std::runtime_error("Failed to get ticker");
    }
    auto biditer = iter->second._buy.rbegin();
    auto askiter = iter->second._sell.begin();
    double bid = biditer->first;
    double ask = askiter->first;
    iter->second._expires = newexp; 
    return Ticker{
        bid,ask,(bid+ask)*0.5,
                static_cast<std::uint64_t>(
                        std::chrono::duration_cast<std::chrono::milliseconds>(httpc.now().time_since_epoch()).count())
    };
}

void CoinbaseAdv::onInit() {
}

json::Value CoinbaseAdv::getSettings(const std::string_view &pairHint) const {
    return {};
}

IStockApi::Orders CoinbaseAdv::getOpenOrders(const std::string_view &pair) {
    std::string p(pair);
    std::unique_lock _(_ws_mx);
    if (_openOrders.empty()) {
        ws.regHandler([=](WsInstance::EventType event, json::Value data){
            std::unique_lock _(_ws_mx);
           switch (event) {
               case WsInstance::EventType::connect: {
                   std::vector<std::string_view> p;
                   std::transform(_openOrders.begin(), _openOrders.end(), std::back_inserter(p),
                      [](const auto &x) -> std::string_view {
                       return x.first; 
                   });
                   if (!p.empty()) {
                       ws_subscribe(false, p, "user");
                       return true;
                   } else {
                       return false;
                   }                     
               }break;
               case WsInstance::EventType::exception:
               case WsInstance::EventType::disconnect: {
                   for (auto &c: _openOrders) {
                       if (c.second._waiter.has_value()) {
                           c.second._wait_error = true;
                           c.second._waiter->set_value();
                           c.second._waiter.reset();
                       }
                   }
               }break;
               case WsInstance::EventType::data: {
                   std::cerr << data.stringify() << std::endl;
                   if (data["channel"].getString() == "users") {
                       for (json::Value event: data["events"]) {
                           std::string_view product = event["product_id"].getString();
                           auto iter = _openOrders.find(product);
                           if (iter == _openOrders.end() || iter->second._expires < std::chrono::system_clock::now()) {
                               ws.send(ws_subscribe(true, {product}, "user"));
                               if (iter != _openOrders.end()) _openOrders.erase(iter);
                               continue;                                
                           }   
                           OrderInfo &o = iter->second;
                           for (json::Value update: event["updates"]) {
                               std::string_view status = update["status"].getString();
                               std::string id = update["order_id"].getString();
                               if (status != "OPEN") {
                                   _openOrders.erase(id);
                               } else {
                                   Order &order = o._orders[id];
                                   order.id = id;
                                   order.client_id = update["client_order_id"];
                                   order.size = update["leaves_quantity"].getNumber();                                   
                               }
                           }
                       }
                   } 
 
               }break;
               
           } 
           return !_openOrders.empty();
        });
    }
    auto newexp = std::chrono::system_clock::now()+std::chrono::minutes(2);
    auto iter = _openOrders.find(p);
    if (iter == _openOrders.end()) {
        auto &o = _openOrders[std::string(p)];
        o._waiter.emplace();
        o._expires = newexp;        
        auto f = o._waiter->get_future();
        bool ok;
        _.unlock();
        ws.send(ws_subscribe(false,{p}, "user"));
        ok =  f.wait_for(std::chrono::seconds(5)) ==std::future_status::ready; 
        if (!ok) {
            ws.send(ws_subscribe(true,{p}, "user"));
        }
        _.lock();
        if (!ok) {
            _openOrders.erase(std::string(p));            
            throw std::runtime_error("Failed to get orders");
        }
    }
    iter = _openOrders.find(p);
    if (iter == _openOrders.end()) {
        throw std::runtime_error("Failed to get ticker");
    }
    Orders res;
    std::transform(iter->second._orders.begin(), 
                    iter->second._orders.end(),
                    std::back_inserter(res),
                    [](const auto &x){return x.second;});
    return res;
}

IStockApi::MarketInfo CoinbaseAdv::getMarketInfo(const std::string_view &pair) {
    if (!fee.has_value()) {
        json::Value v = GET("/api/v3/brokerage/transaction_summary",json::Object{
            {"user_native_currency","USD"},
            {"product_type","SPOT"}
        });
        fee = v["fee_tier"]["maker_fee_rate"].getNumber();
    }
    
    
    std::string url = "/api/v3/brokerage/products/";
    url.append(pair);
    json::Value v = GET(url, json::Value());
    double price = v["price"].getNumber();
    double min_size = v["base_min_size"].getNumber();
    double min_volume = min_size*price;
    return {
        /*asset_symbol*/ v["base_currency_id"].getString(),
        /*currency_symbol*/ v["quote_currency_id"].getString(),
        /*asset_step*/ v["base_increment"].getNumber(),
        /*currency_step*/ v["quote_increment"].getNumber(),
        /*min_size*/ min_size,
        /*min_volume*/ min_volume,
        /*fees*/    *fee,
        /*feeScheme*/ FeeScheme::currency,
        /*leverage*/ 0,
        /*invert_price*/ false,
        /*inverted_symbol*/ std::string(),
        /*simulator */  false,
        /*private_chart*/ false,
        /*wallet_id*/  "spot"
    };
}

uint64_t CoinbaseAdv::downloadMinuteData(const std::string_view &asset,
        const std::string_view &currency, const std::string_view &hint_pair,
        uint64_t time_from, uint64_t time_to,
        std::vector<IHistoryDataSource::OHLC> &data) {
}

IBrokerControl::AllWallets CoinbaseAdv::getWallet() {
    Wallet w;
    w.walletId = "spot";    
    json::Value v = GET("/api/v3/brokerage/accounts", json::Object{{"limit",250}});
    for (json::Value x:v["accounts"]) {
        w.wallet.push_back({
            x["currency"].getString(),
            x["available_balance"]["value"].getNumber()
        });
    }
    return {
        w
    };
}

IStockApi::TradesSync CoinbaseAdv::syncTrades(json::Value lastId, const std::string_view &pair) {
    return {};
}

void CoinbaseAdv::onLoadApiKey(json::Value keyData) {
    api_key = keyData["key"].getString();
    api_secret = keyData["secret"].getString();
    if (!api_key.empty() && !api_secret.empty()) {
        user_details = GET("/v2/user", json::Value())["data"];
/*        ws.regMonitor([this](WsInstance::EventType event, json::Value){
           if (event == WsInstance::EventType::connect) {
               auto tm = std::chrono::duration_cast<std::chrono::seconds>(httpc.now().time_since_epoch()).count();
               json::Value req = json::Object {
                       {"type","users"},
                       {"user_id", user_details["id"]},
                       {"api_key",api_key},
                       {"timestamp",json::Value(tm).toString()},
                       {"signature",calculate_signature(tm, user_details["id"].getString(), "", "")}
               };
               std::cerr << req.toString() << std::endl;
               ws.send(req);
           } 
           return true;
        });*/
    }
    
}

json::Value CoinbaseAdv::testCall(const std::string_view &method,
        json::Value args) {
    if (method == "probeKeys") {
        probeKeys();        
    } else if (method == "GET") {
        if (args.defined()) {
            std::string url = args["command"].getString();
            json::Value query = args["query"];
            return GET(url, query);
        } else {
            return {"GET",json::Object{{"command",""},{"query",json::object}}};
        }
    } else if (method == "POST") {
        if (args.defined()) {
            std::string url = args["command"].getString();
            json::Value query = args["body"];
            return GET(url, query);
        } else {
            return {"POST",json::Object{{"command",""},{"body",json::object}}};
        }
    
    } else {
        return {"probeKeys","GET","POST"};
    }
    
    return json::Value();
}

double CoinbaseAdv::getBalance(const std::string_view &symb, const std::string_view &pair) {
    auto iter = balance_cache.find(symb);
    if (iter != balance_cache.end()) return iter->second;
    auto uuid = get_wallet_uuid(symb);
    if (uuid.empty()) return 0;
    std::string path = "/api/v3/brokerage/accounts/";
    path.append(uuid);
    json::Value v = GET(path,json::Value());
    double b = v["account"]["available_balance"]["value"].getNumber();
    balance_cache.emplace(std::string(symb), b);
    return b;
}

json::Value CoinbaseAdv::setSettings(json::Value v) {
}

void CoinbaseAdv::restoreSettings(json::Value v) {
}

bool CoinbaseAdv::reset() {
    
}

std::string CoinbaseAdv::get_wallet_uuid(std::string_view currency) {
    auto iter = cur_uuid.find(currency);
    if (iter == cur_uuid.end()) {
        
        json::Value v = GET("/api/v3/brokerage/accounts", json::Object{{"limit",250}});
        for (json::Value x:v["accounts"]) {
            cur_uuid[std::string(x["currency"].getString())] = x["uuid"].getString();
        }        
        
        iter = cur_uuid.find(currency);
    }
    if (iter == cur_uuid.end()) {
        return std::string();
    } else {
        return iter->second;
    }
}

json::Value CoinbaseAdv::ws_subscribe(bool unsubscribe, std::vector<std::string_view> products, std::string_view channel) {
    auto tm = std::chrono::duration_cast<std::chrono::seconds>(httpc.now().time_since_epoch()).count();
    std::ostringstream plist;
    json::Array jpl;
    for (std::size_t i = 0; i < products.size(); ++i) {
        if (i) plist.put(',');
        plist << products[i];
        jpl.push_back(products[i]);
    }

    json::Value v = json::Object{
        {"type",unsubscribe?"unsubscribe":"subscribe"},
        {"product_ids",jpl},
        {"channel",channel},
        {"api_key",api_key},
        {"timestamp",json::Value(tm).toString()},
        {"signature",calculate_signature(tm, channel, plist.str(), "")}
    };    
    if  (debug_mode) {
        std::cerr << v.toString() << std::endl;
    }
    return v;
}

json::Value CoinbaseAdv::MyWsInstance::generate_headers() {
    return json::Value();//_owner.headers("GET", "", "");
}

CoinbaseAdv::MyWsInstance::MyWsInstance(CoinbaseAdv &owner,
        simpleServer::HttpClient &client, std::string url)
:WsInstance(client, url), _owner(owner) 
{
}
