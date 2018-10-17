#pragma once

/* ------------------------------------------------------------ *
File RingBuff.hpp
キューをリングバッファを利用して実装したテンプレート
--注意1--
コンストラクタに渡す引数でバッファをこのクラスが用意するのか, 予め
用意した配列を使うのかを決定可能.
コンストラクタにバッファのサイズのみを渡した場合はコンストラクタ内で
配列の確保が行われる.  flag_own_arrayがtrueとなり, デストラクタで
確保した配列がdeleteされる.
コンストラクタに予め確保しておいた配列のポインタとバッファサイズを渡した
場合はflag_own_arrayはfalseとなり, デストラクタでは何も行われない.
--注意2--
空のキューからの取り出しに対するエラー処理はなし
満タンのキューへの要素の追加に対するエラー処理はなし
キューを使う側でisEmpty()とisFull()でキューの状態を確かめること
* ------------------------------------------------------------ */


#include <stdint.h>
/*
template <class T>
class RingBuff{
private:
    T *data;
    uint16_t head;
    uint16_t tail;
    uint16_t max_size;
    uint16_t getNextIndex(uint16_t index);
    bool flag_own_array;
public:
    RingBuff(uint16_t size);
    RingBuff(T *array, uint16_t size);
    ~RingBuff();
    void clearQueue();
    bool isEmpty();
    bool isFull();
    uint16_t getNumElementsNow();
    void enqueue(const T& element);
    T& dequeue();
    uint16_t& getHead();
    uint16_t& getTail();
    T* getDataArray();
};


//次の要素の添え字を求める
template <class T>
uint16_t RingBuff<T>::getNextIndex(uint16_t index)
{
    return (index + 1) % max_size;
}

//バッファサイズを指定し, 自前でバッファを確保する
template <class T>
RingBuff<T>::RingBuff(uint16_t size){
    flag_own_array = true;

    data = new T[size];
    head = 0;
    tail = 0;
    max_size = size;
}

//予め確保しておいたバッファを使用する
template <class T>
RingBuff<T>::RingBuff(T* array, uint_16 size){
    flag_own_array = false;
    data = array;
    head = 0;
    tail = 0;
    max_size = size;
}


//デストラクタ
template <class T>
RingBuff<T>::~RingBuff()
{
    if (flag_own_array == true && data != NULL) delete[] data;
}
//キューを初期化する
template <class T>
void RingBuff<T>::clearQueue()
{
    head = 0;
    tail = 0;
}

//キューが空かどうかの判定をする
template <class T>
bool RingBuff<T>::isEmpty()
{
    return head == tail;
}
//キューが満タンかどうかの判定をする
template <class T>
bool RingBuff<T>::isFull()
{
    return getNextIndex(tail) == head;
}

//キューに入っている現在の要素数を返す
template <class T>
uint16_t RingBuff<T>::getNumElementsNow()
{
    int num = (tail - head + max_size) % max_size;
    return num;
}

//キューから要素を取り出す
template <class T>
T& RingBuff<T>::dequeue(){
    T element;
    element = data[head];
    head = getNextIndex(head);
    return element;
}


//キューに要素を追加する
template <class T>
void RingBuff<T>::enqueue(const T &element){
    data[tail] = element;
    tail = getNextIndex(tail);
}

//headの参照を返す
template <class T>
uint16_t& RingBuff<T>::getHead(){
    return &head;
};

//headの参照を返す
template <class T>
uint16_t& RingBuff<T>::getTail(){
    return &tail;
}

//データを格納している配列の参照を返す
template <class T>
T* getDataArray(){
    return data;
};

*/

//
