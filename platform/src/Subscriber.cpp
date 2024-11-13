/*
LULUNAV
Copyright (C) {{ 2024 }}  {{ yulu_zhong }}

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

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