#include <queue>
#include <mutex>

#pragma once

template<typename T>
class Queue {

    public:
        std::queue<T> queue_;
        std::mutex self_mutex;
        std::mutex size_mutex;
        int size_limit;
        int size;
        Queue();

        void push(T item);

        T read();

        void pop();

        void remove();

        void set_limit(int n);

        
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

    self_mutex.lock();
    auto ret = queue_.front();
    self_mutex.unlock();
    return ret;

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

template <typename T>
void Queue<T>::remove() {
    while (size == 0) {}
    self_mutex.lock();
    queue_.pop();
    size--;
    self_mutex.unlock();
}

template <typename T>
void Queue<T>::set_limit(int n) {
    size_limit = n;
}

