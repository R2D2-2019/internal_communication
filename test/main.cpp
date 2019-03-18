#include "ostream"

#include <ringbuffer.hpp>
#include <queue.hpp>
#include <mock_bus.hpp>

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

TEST_CASE("Queue stores POD data", "[queue]") {
    queue_c<int, 16> q;

    q.push(12);
    q.push(24);

    REQUIRE(q.copy_and_pop() == 12);
    REQUIRE(q.copy_and_pop() == 24);
}

TEST_CASE("Queue stores non-POD data", "[queue]") {
    struct x {
        int a;

        x() : a(0) {}
        x(int a) : a(a) {}

        virtual void foo() {};
    };

    static_assert(! std::is_pod_v<x>);

    queue_c<x, 16> q;

    q.push(x(3));
    q.push(x(5));

    REQUIRE(q.copy_and_pop().a == 3);
    REQUIRE(q.copy_and_pop().a == 5);
}

TEST_CASE("Queue front and back work", "[queue]") {
    queue_c<int, 16> q;

    q.push(12);
    q.push(24);

    REQUIRE(q.front() == 12);
    REQUIRE(q.back() == 24);
}

TEST_CASE("Queue popping an empty queue works", "[queue]") {
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

TEST_CASE("Mock bus accepts packet", "[mock_bus]") {
    mock_comm_c comm;

    comm.listen_for_frames({
        frame_type::GET_DISTANCE
    });

    REQUIRE(comm.accepts_frame(frame_type::GET_DISTANCE));
}

TEST_CASE("Mock bus provides data", "[mock_bus]") {
    mock_comm_c comm;

    comm.listen_for_frames({
        frame_type::GET_DISTANCE
    });

    const auto frame = comm.create_frame<frame_type::GET_DISTANCE>({ 404 });

    comm.accept_frame(frame);

    auto from_comm = comm.get_data();

    REQUIRE(from_comm.type == frame_type::GET_DISTANCE);
    REQUIRE(from_comm.as_frame_type<frame_type::GET_DISTANCE>().mm == 404);
}