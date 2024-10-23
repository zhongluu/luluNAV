#ifndef SUBSCRIBER_
#define SUBSCRIBER_

#include "Manager.hpp"
#include <string>
#include <memory>
#include <unordered_map>
#include <mutex>
#include <thread>

class Subscriber : public std::enable_shared_from_this<Subscriber> {
public:
    virtual ~Subscriber() = default;

    void subscribe(const std::string& topicName);
    void listen();

protected:
    virtual void handleData(const std::string& topicName, std::shared_ptr<void> data) = 0;

private:
    std::unordered_map<std::string, std::shared_ptr<Topic>> topics_;
    std::mutex mtx_;
};

#endif // SUBSCRIBER_HPP