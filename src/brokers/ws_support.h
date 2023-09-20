#pragma once
#ifndef SRC_BROKERS_WS_SUPPORT_H_
#define SRC_BROKERS_WS_SUPPORT_H_

#include <imtjson/value.h>
#include <simpleServer/http_client.h>
#include <simpleServer/websockets_stream.h>

#include <future>
#include <memory>
#include <thread>

class WsInstance {
public:


    enum class EventType {
        data,       //<data arrived
        exception,      //<exception arrived
        connect,      //<connect happen - you probably need to resubscribe
        disconnect,     //<disconnect happen - probably to fail waitings
    };

    using Handler = std::function<bool(EventType event, json::Value data)>;

    WsInstance(simpleServer::HttpClient &client, std::string wsurl);
    WsInstance(const WsInstance &other) = delete;
    WsInstance &operator=(const WsInstance &other) = delete;
    virtual ~WsInstance();


    virtual json::Value generate_headers() {return json::Value();};
    virtual void on_ping();
    void on_connect();
    void on_disconnect();

    void regHandler(Handler &&h);
    void regMonitor(Handler &&h);    //monitor just monitors data and events,

    void worker(std::promise<std::exception_ptr> *start_p);

    virtual void send(json::Value v);


protected:
    simpleServer::HttpClient &_client;
    std::string _wsurl;
    simpleServer::WebSocketStream _ws;
    std::recursive_mutex _mx;
    std::vector<Handler> _handlers;
    std::vector<Handler> _monitors;
    std::thread _thr;
    bool _running = false;

    virtual void broadcast(EventType event, const json::Value &data);
    void process_message(std::string_view msg);
    void ensure_start(std::unique_lock<std::recursive_mutex> &lk);

    void close_lk(std::unique_lock<std::recursive_mutex> &lk);
};




#endif /* SRC_BROKERS_WS_SUPPORT_H_ */
