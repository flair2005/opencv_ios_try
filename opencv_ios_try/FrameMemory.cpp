#include "FrameMemory.hpp"
#include "Frame.hpp"

FrameMemory::FrameMemory()
{
}

FrameMemory& FrameMemory::getInstance()
{
    static FrameMemory theOneAndOnly;
    return theOneAndOnly;
}

void FrameMemory::releaseBuffes()
{
    int total = 0;
    for(auto p : availableBuffers)
    {
        total += p.second.size() * p.first;
        
        for(unsigned int i=0;i<p.second.size();i++)
        {
            Eigen::internal::aligned_free(p.second[i]);
            bufferSizes.erase(p.second[i]);
        }
        
        p.second.clear();
    }
    availableBuffers.clear();
}


void* FrameMemory::getBuffer(unsigned int sizeInByte)
{
    if (availableBuffers.count(sizeInByte) > 0)
    {
        std::vector< void* >& availableOfSize = availableBuffers.at(sizeInByte);
        if (availableOfSize.empty())
        {
            void* buffer = allocateBuffer(sizeInByte);
            return buffer;
        }
        else
        {
            void* buffer = availableOfSize.back();
            availableOfSize.pop_back();
            return buffer;
        }
    }
    else
    {
        void* buffer = allocateBuffer(sizeInByte);
        return buffer;
    }
}

float* FrameMemory::getFloatBuffer(unsigned int size)
{
    return (float*)getBuffer(sizeof(float) * size);
}

void FrameMemory::returnBuffer(void* buffer)
{
    if(buffer==0) return;
    unsigned int size = bufferSizes.at(buffer);
    if (availableBuffers.count(size) > 0)
        availableBuffers.at(size).push_back(buffer);
    else
    {
        std::vector< void* > availableOfSize;
        availableOfSize.push_back(buffer);
        availableBuffers.insert(std::make_pair(size, availableOfSize));
    }
}

void* FrameMemory::allocateBuffer(unsigned int size)
{
    void* buffer = Eigen::internal::aligned_malloc(size);
    bufferSizes.insert(std::make_pair(buffer, size));
    return buffer;
}

void FrameMemory::activateFrame(Frame* frame)
{
}

void FrameMemory::deactivateFrame(Frame* frame)
{
}

void FrameMemory::pruneActiveFrames()
{
}
