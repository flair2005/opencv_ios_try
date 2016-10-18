
#ifndef FrameMemory_hpp
#define FrameMemory_hpp

#include <stdio.h>
#include <Eigen/Core> 
#include <unordered_map>
#include <vector>
#include <list>

class Frame;
class FrameMemory
{
public:
    /** Returns the global instance. Creates it when the method is first called. */
    static FrameMemory& getInstance();
    
    /** Allocates or fetches a buffer with length: size * sizeof(float).
     * Corresponds to "buffer = new float[size]". */
    float* getFloatBuffer(unsigned int size);
    
    /** Allocates or fetches a buffer with length: size * sizeof(float).
     * Corresponds to "buffer = new float[size]". */
    void* getBuffer(unsigned int sizeInByte);
    
    /** Returns an allocated buffer back to the global storage for re-use.
     * Corresponds to "delete[] buffer". */
    void returnBuffer(void* buffer);
    
    void activateFrame(Frame* frame);
    void deactivateFrame(Frame* frame);
    void pruneActiveFrames();
    
    void releaseBuffes();
private:
    FrameMemory();
    void* allocateBuffer(unsigned int sizeInByte);

    std::unordered_map< void*, unsigned int > bufferSizes;
    std::unordered_map< unsigned int, std::vector< void* > > availableBuffers;
    
    std::list<Frame*> activeFrames;
};

#endif /* FrameMemory_hpp */
