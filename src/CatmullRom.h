#ifndef CatmullRom_h
#define CatmullRom_h

// CatmullRom class is for catmull rom interpolation. 
class CatmullRom
{
public:
    // This is the step size near the known data points. It isn't ideal since target step is defined, but it was a quick fix.
    static constexpr float END_STEP_SIZE = 1;

    CatmullRom();
    CatmullRom(float* pathX, float* pathY, int numPoints);
    void setPoints(float* pathX, float* pathY, int numPoints);

    bool hasNext();
    bool hasPrev();
    bool next();
    bool prev();
    float getIterationX();
    float getIterationY();
    void resetIterator(float stepSize);
    
    void printPath();
    float getX(int i);
    float getY(int i);
    float* getArrayX();
    float* getArrayY();
    int getLength();
    float static getDistance(float x1, float y1, float x2, float y2);
    float static catmullRomInterpolation(float* ary, int i, float t);
    void interpolate(int i, float t, float* x, float* y);
    float getParameterStep(int i, float targetStep);

private:
    float* _pathX;
    float* _pathY;
    int _numPoints;

    float _stepSize;
    float _parameterStepSize;
    float _iterationX;
    float _iterationY;
    int _iterationArrayIndex;
    int _iterationIndex;
    float _t;
};



#endif
