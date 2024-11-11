#include "Subscriber.hpp"

void Subscriber::subscribe(const std::string& topicName) 
{
    auto topic = TopicManager::getInstance().getTopic(topicName);
    if (!topic) {
        TopicManager::getInstance().registerTopic(topicName);
        topic = TopicManager::getInstance().getTopic(topicName);
    }

    {
        std::lock_guard<std::mutex> lock(topic->mtx);
        topic->subscribers.push_back(shared_from_this());
    }

    {
        std::lock_guard<std::mutex> lock(mtx_);
        topics_[topicName] = topic;
    }
}

void Subscriber::listen() 
{
    while (true) {
        for (const auto& topic_pair : topics_) {
            const auto& name = topic_pair.first;
            const auto& topic = topic_pair.second;
            // int currentFlag;
            std::shared_ptr<void> data;
            {
                // std::lock_guard<std::mutex> lock(topic->mtx);
                topic->mtx.lock();
                // currentFlag = topic->updateFlag;
                if (topic->updateFlag > 0) {
                    if (!topic->dataQueue.empty()) {
                        data = topic->dataQueue.front();
                        topic->dataQueue.pop();
                        topic->updateFlag--;
                        topic->mtx.unlock();
                    }
                    handleData(name, data);
                } else {
                    topic->mtx.unlock();
                }
                
            }
        }

        // std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}