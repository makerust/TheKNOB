template<typename _Tp, int LENGTH>
struct mailbox {
  typedef _Tp               value_type;
  typedef const value_type& const_reference;
  typedef size_t            size_type;

  value_type buf[LENGTH ? LENGTH : 1];
  size_type head;   // Write To Head
  size_type tail;   // Read from Tail
  bool full;

  void
  fill(const value_type& v) {
    for (auto i = 0; i < LENGTH; i++) buf[i] = v;
    head = 0;
    tail = 0;
    full = false;
  }

  size_type
  count() const{
    if (full) return LENGTH;
    return (head >= tail) ? head - tail
                          : LENGTH + head - tail;
  }

  void
  push_back(const_reference value) {
    buf[head] = value;
    if (full){
      if (++tail == LENGTH) tail = 0;
    }
    if (++head == LENGTH) head = 0;
    full = head == tail;
  }

  value_type
  pop_front(){
    auto res = buf[tail];
    full = false;
    if (++tail >= LENGTH) tail = 0;
    return res;
  }

  bool
  is_full() const{
    return full;
  }

};
