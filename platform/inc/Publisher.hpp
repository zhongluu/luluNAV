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
#ifndef PUBLISHER_
#define PUBLISHER_

#include "Manager.hpp"
#include <vector>
#include <memory>

class Publisher {
public:
    virtual ~Publisher() = default;

    void setTopics(const std::string& topicName);

    void publish(std::shared_ptr<void> data);

private:
    std::vector<std::shared_ptr<Topic>> topicsToPublish_;
};

#endif // PUBLISHER_HPP