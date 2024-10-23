#include "Manager.hpp"

TopicManager& TopicManager::getInstance() {
    static TopicManager instance;
    return instance;
}

std::shared_ptr<Topic> TopicManager::getTopic(const std::string& topicName) {
    std::lock_guard<std::mutex> lock(mtx_);
    auto it = topics_.find(topicName);
    if (it != topics_.end()) {
        return it->second;
    }
    return nullptr;
}

void TopicManager::registerTopic(const std::string& topicName) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (topics_.find(topicName) == topics_.end()) {
        topics_[topicName] = std::make_shared<Topic>();
        topics_[topicName]->name = topicName;
    }
}