#pragma once

// Only used with the driver disabled. ISR records are dumped after capture ends.
class ManualDiagnostics {
 public:
  struct Edge { uint32_t us; int32_t count; uint8_t state; };
  struct Index { uint32_t us, a, b; int32_t count; };
  static constexpr uint16_t capacity = 256;
  volatile bool active = false;
  volatile int32_t count = 0;
  volatile uint32_t invalid = 0, duplicate = 0, reversals = 0;
  volatile uint32_t intervals[4] = {}, sumUs[4] = {}, minUs[4] = {}, maxUs[4] = {};
  volatile Edge edges[capacity];
  volatile Index indices[16];
  volatile uint16_t next = 0, size = 0, indexSize = 0;
  volatile uint32_t indexOverflow = 0;
  uint32_t startUs = 0;
 private:
  uint8_t previous = 0;
  int8_t direction = 0;
  uint32_t lastEdgeUs = 0;
  bool haveEdge = false;
 public:
  void begin(uint8_t state) {
    // Called with interrupts disabled, while inactive.
    count = 0; invalid = duplicate = reversals = 0;
    next = size = indexSize = 0; indexOverflow = 0;
    for (int i = 0; i < 4; ++i) {
      intervals[i] = sumUs[i] = maxUs[i] = 0;
      minUs[i] = UINT32_MAX;
    }
    previous = state; direction = 0; haveEdge = false;
    startUs = micros();
    active = true;
  }

  void edge(uint8_t state) {
    if (!active) return;
    const uint32_t now = micros();
    const uint8_t changed = previous ^ state;
    static constexpr int8_t steps[16] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};
    const int8_t step = steps[previous * 4 + state];
    if (changed == 3) ++invalid;
    else if (!changed) ++duplicate;
    else {
      count += step;
      if (direction && step != direction) ++reversals;
      direction = step;
      if (haveEdge) {
        const uint32_t dt = now - lastEdgeUs;
        ++intervals[previous]; sumUs[previous] += dt;
        if (dt < minUs[previous]) minUs[previous] = dt;
        if (dt > maxUs[previous]) maxUs[previous] = dt;
      }
    }
    // Invalid/duplicate callbacks are retained too; these are software observations.
    edges[next].us = now - startUs;
    edges[next].count = count;
    edges[next].state = state;
    next = (next + 1) % capacity;
    if (size < capacity) ++size;
    previous = state; lastEdgeUs = now; haveEdge = true;
  }

  void index(uint32_t a, uint32_t b) {
    if (!active) return;
    if (indexSize == 16) { ++indexOverflow; return; }
    indices[indexSize].us = micros() - startUs;
    indices[indexSize].a = a; indices[indexSize].b = b;
    indices[indexSize].count = count;
    ++indexSize;
  }

  void dump(Stream& out) {
    // active has already been cleared; ISR no longer changes these records.
    out.print("MANUAL_SUMMARY count,invalid,duplicate,reversals,index_overflow: ");
    out.print(count); out.print(','); out.print(invalid); out.print(',');
    out.print(duplicate); out.print(','); out.print(reversals); out.print(','); out.println(indexOverflow);
    out.println("EDGE_STATS previous_AB,samples,min_us,max_us,mean_us");
    for (int i = 0; i < 4; ++i) {
      out.print(i); out.print(','); out.print(intervals[i]); out.print(',');
      out.print(intervals[i] ? minUs[i] : 0); out.print(','); out.print(maxUs[i]); out.print(',');
      out.println(intervals[i] ? float(sumUs[i]) / intervals[i] : 0, 3);
    }
    out.println("INDEX us,A_edges,B_edges,signed_count");
    for (uint16_t i = 0; i < indexSize; ++i) {
      out.print(indices[i].us); out.print(','); out.print(indices[i].a); out.print(',');
      out.print(indices[i].b); out.print(','); out.println(indices[i].count);
    }
    out.println("EDGES_LAST us,signed_count,AB_state");
    for (uint16_t i = 0; i < size; ++i) {
      const uint16_t at = (next + capacity - size + i) % capacity;
      out.print(edges[at].us); out.print(','); out.print(edges[at].count); out.print(','); out.println(edges[at].state);
    }
    out.println("MANUAL_END: gates OFF; drive locked until reset.");
  }
};
