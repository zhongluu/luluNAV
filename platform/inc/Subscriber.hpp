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