#pragma once
#include <string>
#include <mutex>
#include <openssl/hmac.h>
#include <openssl/evp.h>
class LedScreen
{
public:

    static LedScreen& Instance();

    bool Init(const std::string& ip,
              int port,
              const std::string& sdkKey,
              const std::string& sdkSecret,
              const std::string& deviceId);

    bool ShowText(const std::string& text);

private:

    LedScreen() = default;

    bool HttpPost(const std::string& body);
    std::string GetGMTTime();
    std::string GenerateRequestId();
    std::string HmacMD5(const std::string& data, const std::string& key);
    std::string MaskKey(const std::string& key);
    bool IsSuccessResponse(const std::string& response);

private:

    std::string ip_;
    int port_;
    std::string sdkKey_;
    std::string deviceId_;
    std::string sdkSecret_ ;
    std::mutex mutex_;
};
