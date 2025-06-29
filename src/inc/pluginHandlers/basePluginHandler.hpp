#ifndef BASEPLUGINHANDLER_H
#define BASEPLUGINHANDLER_H

#include <string>

class BasePluginHandler
{
public:
  virtual int pluginInit(std::string &pluginURI) = 0;
  virtual int pluginActivateInstance() = 0;
  virtual int pluginRun(int sampleRate) = 0;

  // Adapter to connect to different port types
  virtual int pluginConnectPort(int portIdx, float &var) = 0;
  virtual int pluginConnectPort(int portIdx, int &var) = 0;
  // BasePluginHandler();
  // virtual ~BasePluginHandler() = default;
};


#endif /* BASEPLUGINHANDLER_H */
