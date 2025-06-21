#ifndef BASEPLUGINHANDLER_H
#define BASEPLUGINHANDLER_H

class BasePluginHandler
{
public:
  virtual int pluginInit(std::string &pluginURI) = 0;
  BasePluginHandler();
  virtual ~BasePluginHandler() = default;
};


#endif /* BASEPLUGINHANDLER_H */
