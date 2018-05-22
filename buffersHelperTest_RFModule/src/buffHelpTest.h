

#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include "BuffersHelper.h"
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

class myThread : public yarp::os::RateThread
{
public:

    BuffersHelper<int> * buffsHelper;
    int min,max, count, id;

    myThread():RateThread(100){;}
    void init(BuffersHelper<int> * buffs, int num)
    {
        buffsHelper = buffs;
        min=10*num;
        max=10*(num+1) -1;
        count = min;
        setRate(10+4*num);
        id = num;
    }
    myThread(BuffersHelper<int> * buffs, int num):RateThread(100+8*num)
    {
        buffsHelper = buffs;
        min=10*num;
        max=10*(num+1) -1;
        count = min;
    }

    void run(void)
    {
        int *myBuff= buffsHelper->get_buffer();
        for(int i=0; i<buffsHelper->get_buffers_size(); i++)
            myBuff[i] = count;
        yWarning() << "TH " << id << ": I'm using buff " << myBuff;
        count++;
        if(count==max)
            count=min;

        buffsHelper->release_buffer(myBuff);
        yWarning() << "TH " << id << ": I released buff " << myBuff;
    }
};





class BuffHelpTest:public yarp::os::RFModule
{
public:
    static const int8_t NUMJOINTS=15;
    static const int8_t NUM_THREADS=10;
    BuffHelpTest();
    ~BuffHelpTest();

    double getPeriod();

    bool updateModule();

    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);

    bool configure(yarp::os::ResourceFinder &rf);
    // Interrupt function.
    bool interruptModule();

    bool close();
private:
    BuffersHelper<int> buffsHelper;
    myThread threadList[NUM_THREADS];


};