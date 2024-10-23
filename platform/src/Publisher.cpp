#include "Publisher.hpp"

void Publisher::setTopics(const std::string& topicName) 
{
    auto topic = TopicManager::getInstance().getTopic(topicName);
    if (!topic) {
        TopicManager::getInstance().registerTopic(topicName);
        topic = TopicManager::getInstance().getTopic(topicName);
        topicsToPublish_.push_back(topic);
    } else {
        topicsToPublish_.push_back(topic);
    }
}

void Publisher::publish(std::shared_ptr<void> data) 
{
    for (const auto& topic : topicsToPublish_) {
        // std::lock_guard<std::mutex> lock(topic->mtx);
        topic->mtx.lock();
        topic->dataQueue.push(data);
        topic->updateFlag++;
        topic->mtx.unlock();
    }
}
