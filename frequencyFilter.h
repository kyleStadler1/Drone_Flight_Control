class frequencyFilter {
 
  private:
    float *r;
    float *p;
    float *y;
    unsigned int filterSize;
  public:
    frequencyFilter(unsigned int filterSize) {
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
      r[i] = r[i-1];
      p[i] = p[i-1];
      y[i] = y[i-1];
    }
    r[0] = rollRaw;
    p[0] = pitchRaw;
    y[0] = yawRaw;
    float rollMin; float rollMax; float pitchMin; float pitchMax; float yawMin; float yawMax;
    minMax(&rollMin, &rollMax, &pitchMin, &pitchMax, &yawMin, &yawMax);
    *rollFilt = (rollMin + rollMax)/2.0;
    *pitchFilt = (pitchMin + pitchMax)/2.0;
    *yawFilt = (yawMin + yawMax)/2.0;
  }


  void resetFilter() {
    for (int i = 0; i < filterSize; i++) {
      r[i] = 0.0;
      p[i] = 0.0;
      y[i] = 0.0;
    }
  }

  private:
  void minMax(float *rollMin, float*rollMax, float *pitchMin, float *pitchMax, float *yawMin, float *yawMax) {
    *rollMax = *rollMin = r[0];
    *pitchMax = *pitchMin = p[0];
    *yawMax = *yawMin = y[0];
    for (int i = 0; i < filterSize; i++) {
      float rVal = r[i];
      float pVal = p[i];
      float yVal = y[i];
      *rollMax = rVal > *rollMax ? rVal : *rollMax;
      *rollMin = rVal < *rollMin ? rVal : *rollMax;
      *pitchMax = pVal > *pitchMax ? pVal : *pitchMax;
      *pitchMin = pVal < *pitchMin ? pVal : *pitchMax;
      *yawMax = yVal > *yawMax ? yVal : *yawMax;
      *yawMin = yVal < *yawMin ? yVal : *yawMax;      
    }
  }
  
};
