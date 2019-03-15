#include "ostream"

#include <ringbuffer.hpp>
#include <queue.hpp>

#define CATCH_CONFIG_MAIN
#include <catch.hpp>

using namespace r2d2;

/** RINGBUFFER **/

TEST_CASE("Ringbuffer stores data", "[ringbuffer]") {
    ringbuffer_c<int, 16> buffer;
    buffer.push(5);

    REQUIRE(buffer.copy_and_pop() == 5);
}

TEST_CASE("Ringbuffer shows the correct size", "[ringbuffer]") {
    ringbuffer_c<int, 16> buffer;
    buffer.push(5);

    REQUIRE(buffer.max_size() == 16);
    REQUIRE(buffer.size() == 1);

    buffer.push(92);

    REQUIRE(buffer.size() == 2);
}

TEST_CASE("Ringbuffer copy and pop reduces size", "[ringbuffer]") {
    ringbuffer_c<int, 16> buffer;
    buffer.push(5);
    buffer.copy_and_pop();

    REQUIRE(buffer.empty());
}

TEST_CASE("Ringbuffer full and reset works", "[ringbuffer]") {
    ringbuffer_c<int, 2> buffer;

    buffer.push(5);
    buffer.push(18);

    REQUIRE(buffer.full());

    buffer.reset();

    REQUIRE(buffer.empty());
}

TEST_CASE("Ringbuffer wraps around on overflow", "[ringbuffer]") {
    ringbuffer_c<int, 2> buffer;

    buffer.push(5);
    buffer.push(92);
    buffer.push(18);

    REQUIRE(buffer.copy_and_pop() == 92);
    REQUIRE(buffer.copy_and_pop() == 18);
}

TEST_CASE("Ringbuffer operator[] works", "[ringbuffer]") {
    ringbuffer_c<int, 2> buffer;

    buffer.push(5);
    buffer.push(92);

    REQUIRE(buffer[0] == 5);
    REQUIRE(buffer[1] == 92);

    buffer.push(18);
    REQUIRE(buffer[1] == 18);
}

/** QUEUE **/

TEST_CASE("Queue store data", "[queue]") {
    queue_c<int, 16> q;

    q.push(12);
    q.push(24);

    REQUIRE(q.copy_and_pop() == 12);
    REQUIRE(q.copy_and_pop() == 24);
}

TEST_CASE("Queue front and back work", "[queue]") {
    queue_c<int, 16> q;

    q.push(12);
    q.push(24);

    REQUIRE(q.front() == 12);
    REQUIRE(q.back() == 24);
}

TEST_CASE("Queue popping and empty queue works", "[queue]") {
    queue_c<int, 2> q;

    q.pop();

    REQUIRE(true);
}

TEST_CASE("Queue empty, full, max_size and size work", "[queue]") {
    queue_c<int, 2> q;

    q.push(12);
    q.push(24);

    REQUIRE(q.size() == 2);
    REQUIRE(q.max_size() == 2);
    REQUIRE(q.full());

    q.pop();
    q.pop();

    REQUIRE(q.empty());
}