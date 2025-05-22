// CircularBuffer.h
#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

class CircularBuffer {
private:
    float* buffer;
    int capacity;
    int size;
    int head;
    int tail;

public:
    CircularBuffer(int maxSize) {
        capacity = maxSize;
        buffer = new float[capacity];
        size = 0;
        head = 0;
        tail = 0;
    }

    ~CircularBuffer() {
        delete[] buffer;
    }

    void add(float value) {
        buffer[head] = value;
        head = (head + 1) % capacity;
        
        if (size < capacity) {
            size++;
        } else {
            tail = (tail + 1) % capacity;
        }
    }

    void getFeatures(float* output) {
        for (int i = 0; i < size; i++) {
            int index = (tail + i) % capacity;
            output[i] = buffer[index];
        }
    }

    void clear() {
        size = 0;
        head = 0;
        tail = 0;
    }

    int getSize() {
        return size;
    }
};

#endif
