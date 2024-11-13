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