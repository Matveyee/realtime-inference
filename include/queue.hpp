#include <queue>
#include <mutex>

#pragma once

template<typename T>
class Queue {

    public:
        std::queue<T> queue_;
        std::mutex self_mutex;
        int size;
        Queue();

        void push(T item);

        T read();

        void pop();

        
};

template <typename T>
Queue<T>::Queue() {

    size = 0;

}

template <typename T>
void Queue<T>::push(T item) {
    
    self_mutex.lock();
    queue_.push(item);
    size++;
    self_mutex.unlock();

}

template <typename T>
T Queue<T>::read() {


    return queue_.front();


}

template <typename T>
void Queue<T>::pop() {
    while (size == 0) {}
    self_mutex.lock();
    queue_.front().close();
    queue_.pop();
    size--;
    self_mutex.unlock();
}