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