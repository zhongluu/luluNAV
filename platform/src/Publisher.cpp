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
