//
// Created by ica on 2023/2/19.
//

#ifndef ROUNDQUEUE_H__
#define ROUNDQUEUE_H__


// 循环队列
template<class type, int length>
class RoundQueue {
  private:
    type data[length];
    int head;
    int tail;
  public:
    RoundQueue<type, length>() : head(0), tail(0) {};

    constexpr int size() const {
        return length;
    };

    bool empty() const {
        return head == tail;
    };

    void push(const type &obj) {
        data[head] = obj;
        head = (head + 1) % length;
        if (head == tail) {
            tail = (tail + 1) % length;
        }
    };

    bool pop(type &obj) {
        if (empty()) return false;
        obj = data[tail];
        tail = (tail + 1) % length;
        return true;
    };

    type &operator[](int idx) {
        while (tail + idx < 0) idx += length;
        return data[(tail + idx) % length];
    };
};

#endif // ROUNDQUEUE_H__