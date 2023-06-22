class windowFilter {
 
  private:
    float *r;
    float *p;
    float *y;
    unsigned int filterSize;
  public:
    windowFilter(unsigned int filterSize) {
      this->filterSize = filterSize;
      r = new float[filterSize];
      p = new float[filterSize];
      y = new float[filterSize];
      for (int i = 0; i < filterSize; i++) {
        r[i] = 0.0;
        p[i] = 0.0;
        y[i] = 0.0;
      }
    }
  
    void filter(float rollRaw, float pitchRaw, float yawRaw, float *rollFilt, float *pitchFilt, float *yawFilt) {
    for (int i = filterSize-1; i > 0; i--) {
      r[i] = r[i-1] + rollRaw;
      p[i] = p[i-1] + pitchRaw;
      y[i] = y[i-1] + yawRaw;
    }
    r[0] = rollRaw;
    p[0] = pitchRaw;
    y[0] = yawRaw;
    *rollFilt = r[filterSize-1]/(double)filterSize;
    *pitchFilt = p[filterSize-1]/(double)filterSize;
    *yawFilt = y[filterSize-1]/(double)filterSize;
  }


  void resetFilter() {
    for (int i = 0; i < filterSize; i++) {
      r[i] = 0.0;
      p[i] = 0.0;
      y[i] = 0.0;
    }
  }
  
};
