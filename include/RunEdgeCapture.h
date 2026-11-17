#pragma once
#include <cstdint>
#include <cstddef>

// Borrows inactive manual edge storage; never sets the manual-mode flag.
// begin/freeze are called with IRQs excluded. Only the ISR calls edge().
// A frozen capture must be sent and released before manual storage is reused.
template<class Edge,std::size_t Capacity>
class RunEdgeCapture {
 public:
  static_assert(Capacity>0 && Capacity<=65535,"bounded index representation");
  explicit RunEdgeCapture(volatile Edge (&storage)[Capacity]):storage(storage) {}
  volatile bool active=false,pending=false;
  volatile uint16_t next=0,size=0;
  volatile uint32_t total=0;
  int32_t originCount=0,endCount=0;
  uint32_t endUs=0;
  uint8_t originState=0;
  bool busy() const {return active || pending;}
  bool begin(int32_t count,uint8_t state) {
    if (busy() || state>3) return false;
    next=size=0;total=0;originCount=endCount=count;originState=state;endUs=0;active=true;
    return true;
  }
  void edge(uint32_t us,int32_t count,uint8_t state) {
    if (!active) return;
    storage[next].us=us;storage[next].count=count;storage[next].state=state;
    next=static_cast<uint16_t>((next+1)%Capacity);
    if (size<Capacity) ++size;
    ++total;
  }
  void freeze(uint32_t us,int32_t count) {
    if (!active) return;
    active=false;endUs=us;endCount=count;pending=true;
  }
  bool read(uint16_t index,Edge& output) const {
    if (active || !pending || index>=size) return false;
    const uint16_t at=static_cast<uint16_t>((next+Capacity-size+index)%Capacity);
    output.us=storage[at].us;output.count=storage[at].count;output.state=storage[at].state;
    return true;
  }
  bool release() {
    if (active || !pending) return false;
    pending=false;return true;
  }
 private:
  volatile Edge (&storage)[Capacity];
};
