
#ifndef FrameMemory_hpp
#define FrameMemory_hpp

#include <stdio.h>

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
    
    
    boost::shared_lock<boost::shared_mutex> activateFrame(Frame* frame);
    void deactivateFrame(Frame* frame);
    void pruneActiveFrames();
    
    void releaseBuffes();
private:
    FrameMemory();
    void* allocateBuffer(unsigned int sizeInByte);
    
    boost::mutex accessMutex;
    std::unordered_map< void*, unsigned int > bufferSizes;
    std::unordered_map< unsigned int, std::vector< void* > > availableBuffers;
    
    
    boost::mutex activeFramesMutex;
    std::list<Frame*> activeFrames;
};

#endif /* FrameMemory_hpp */
