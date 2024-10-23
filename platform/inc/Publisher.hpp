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