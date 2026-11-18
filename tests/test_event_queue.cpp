#include <gtest/gtest.h>
#include "../include/engine/EventScheduler.hpp"
#include "../include/events/Event.hpp"
#include <thread>
#include <vector>

using namespace simulator::mesh;

// Dummy event for testing
class TestEvent : public events::Event {
private:
    long long timestamp;
    int priority;
    int id;
public:
    TestEvent(long long timestamp, int priority, int id) 
        : timestamp(timestamp), priority(priority), id(id) {}
        
    void execute() override {}
    long long getTimestamp() const override { return timestamp; }
    int getPriority() const override { return priority; }
    int getId() const { return id; }
};

TEST(EventQueueTest, PriorityAndTimestampOrdering) {
    engine::EventScheduler scheduler;
    
    TestEvent e1(100, 1, 1);
    TestEvent e2(50, 2, 2); // Higher priority
    TestEvent e3(150, 2, 3); // Same priority, later timestamp
    TestEvent e4(10, 2, 4); // Same priority, earlier timestamp
    
    scheduler.scheduleEvent(&e1);
    scheduler.scheduleEvent(&e2);
    scheduler.scheduleEvent(&e3);
    scheduler.scheduleEvent(&e4);
    
    // In Phase 3 (simulated time):
    // Earlier timestamp first, then higher priority, then insertion sequence (FIFO).
    // Expected order:
    // e4 (timestamp 10, priority 2)
    // e2 (timestamp 50, priority 2)
    // e1 (timestamp 100, priority 1)
    // e3 (timestamp 150, priority 2)
    
    auto pop1 = dynamic_cast<TestEvent*>(scheduler.popNextEvent());
    ASSERT_NE(pop1, nullptr);
    EXPECT_EQ(pop1->getId(), 4);
    
    auto pop2 = dynamic_cast<TestEvent*>(scheduler.popNextEvent());
    ASSERT_NE(pop2, nullptr);
    EXPECT_EQ(pop2->getId(), 2);
    
    auto pop3 = dynamic_cast<TestEvent*>(scheduler.popNextEvent());
    ASSERT_NE(pop3, nullptr);
    EXPECT_EQ(pop3->getId(), 1);
    
    auto pop4 = dynamic_cast<TestEvent*>(scheduler.popNextEvent());
    ASSERT_NE(pop4, nullptr);
    EXPECT_EQ(pop4->getId(), 3);
    
    EXPECT_EQ(scheduler.popNextEvent(), nullptr);
}

TEST(EventQueueTest, SameTimestampPriorityAndFifoTieBreaking) {
    engine::EventScheduler scheduler;
    
    // Same timestamp (100)
    TestEvent eLowPri(100, 1, 1);
    TestEvent eHighPri1(100, 5, 2); // Higher priority, inserted first
    TestEvent eHighPri2(100, 5, 3); // Same high priority, inserted second
    TestEvent eMidPri(100, 3, 4);
    
    scheduler.scheduleEvent(&eLowPri);
    scheduler.scheduleEvent(&eHighPri1);
    scheduler.scheduleEvent(&eHighPri2);
    scheduler.scheduleEvent(&eMidPri);
    
    // Expected order at timestamp 100:
    // 1. eHighPri1 (pri 5, seq earlier)
    // 2. eHighPri2 (pri 5, seq later)
    // 3. eMidPri (pri 3)
    // 4. eLowPri (pri 1)
    auto pop1 = dynamic_cast<TestEvent*>(scheduler.popNextEvent());
    ASSERT_NE(pop1, nullptr);
    EXPECT_EQ(pop1->getId(), 2);
    
    auto pop2 = dynamic_cast<TestEvent*>(scheduler.popNextEvent());
    ASSERT_NE(pop2, nullptr);
    EXPECT_EQ(pop2->getId(), 3);
    
    auto pop3 = dynamic_cast<TestEvent*>(scheduler.popNextEvent());
    ASSERT_NE(pop3, nullptr);
    EXPECT_EQ(pop3->getId(), 4);
    
    auto pop4 = dynamic_cast<TestEvent*>(scheduler.popNextEvent());
    ASSERT_NE(pop4, nullptr);
    EXPECT_EQ(pop4->getId(), 1);
    
    EXPECT_EQ(scheduler.popNextEvent(), nullptr);
}


TEST(EventQueueTest, ConcurrentScheduling) {
    engine::EventScheduler scheduler;
    std::vector<TestEvent> events;
    for (int i = 0; i < 1000; ++i) {
        events.emplace_back(i, i % 10, i);
    }
    
    std::vector<std::thread> threads;
    for (int i = 0; i < 10; ++i) {
        threads.emplace_back([&scheduler, &events, i]() {
            for (int j = 0; j < 100; ++j) {
                scheduler.scheduleEvent(&events[i * 100 + j]);
            }
        });
    }
    
    for (auto& t : threads) {
        t.join();
    }
    
    int poppedCount = 0;
    while (scheduler.popNextEvent() != nullptr) {
        poppedCount++;
    }
    EXPECT_EQ(poppedCount, 1000);
}
