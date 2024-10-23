#ifndef TOPIC_MANAGER_
#define TOPIC_MANAGER_

#include <string>
#include <memory>
#include <unordered_map>
#include <mutex>
#include <vector>
#include <queue>

// Forward declaration of Subscriber class
class Subscriber;

// 主题类，包含主题名称、订阅者列表、数据队列和更新标识符
class Topic {
public:
    std::string name;
    std::queue<std::shared_ptr<void>> dataQueue;
    std::vector<std::weak_ptr<Subscriber>> subscribers;
    int updateFlag = 0;
    std::mutex mtx;
};

// TopicManager 单例类，用于管理所有的主题
class TopicManager {
public:
    static TopicManager& getInstance();

    std::shared_ptr<Topic> getTopic(const std::string& topicName);
    void registerTopic(const std::string& topicName);

private:
    TopicManager() = default;
    std::unordered_map<std::string, std::shared_ptr<Topic>> topics_;
    std::mutex mtx_;
};

#endif