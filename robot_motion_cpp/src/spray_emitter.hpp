#pragma once

#include <cstdlib>
#include <string>
// i was traing to run this command using my c++ node to controle the spray state
// gz topic -t /spray/emitter -m gz.msgs.ParticleEmitter -p 'emitting: {data: true}'
// send it to the ai and it make this after some adjustments
// i learn alot from ai, things like faser ways to write the code or in a much cleaner way

namespace spray_controller
{
    class SprayEmitter
    {
    public:
        explicit SprayEmitter(const std::string & topic = "/spray/emitter") : topic_(topic) {}

        ~SprayEmitter() { off(); }

        bool on()  { return set(true); }
        bool off() { return set(false); }

        bool set(bool value)
        {
            const std::string cmd =
                "gz topic -t " + topic_ +
                " -m gz.msgs.ParticleEmitter -p 'emitting: {data: " +
                (value ? "true" : "false") + "}'";     // <===
            return std::system(cmd.c_str()) == 0;
        }

    private:
        std::string topic_;
    };

    //     (value ? "true" : "false")    actually thats smart, i remember this from a C++ course, its hard to keep track of these small details.
    // if i wrote this i would write :
    
    // bool set(bool value)
    // {
    //     std::string data;
    //     if(value){data = "true";}else{data = "false";}

    //     const std::string cmd =
    //         "gz topic -t " + topic_ +
    //         " -m gz.msgs.ParticleEmitter -p 'emitting: {data: " + data + "}'";
    //     return std::system(cmd.c_str()) == 0;
    // };
    
}  // namespace painting_robot