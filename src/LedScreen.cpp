#include "LedScreen.h"
#include <curl/curl.h>
#include <json/json.h>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>

static size_t WriteCallback(void* contents,size_t size,size_t nmemb,void* userp)
{
    std::string* str = (std::string*)userp;
    str->append((char*)contents,size*nmemb);
    return size*nmemb;
}

LedScreen& LedScreen::Instance()
{
    static LedScreen instance;
    return instance;
}
std::string LedScreen::GetGMTTime()
{
    char buf[128];
    time_t t = time(nullptr);
    struct tm *gmt = gmtime(&t);
    strftime(buf, sizeof(buf), "%a, %d %b %Y %H:%M:%S GMT", gmt);
    return std::string(buf);
}

std::string LedScreen::GenerateRequestId()
{
    static std::mt19937_64 rng(std::random_device{}());
    auto now = std::chrono::system_clock::now().time_since_epoch().count();

    std::ostringstream oss;
    oss << std::hex << now << "-" << rng();
    return oss.str();
}

std::string LedScreen::HmacMD5(const std::string& data, const std::string& key)
{
    unsigned char* digest;
    digest = HMAC(EVP_md5(),
                  key.c_str(), key.length(),
                  (unsigned char*)data.c_str(), data.length(),
                  NULL, NULL);

    char md[33] = {0};
    for(int i = 0; i < 16; i++)
        sprintf(&md[i * 2], "%02x", (unsigned int)digest[i]);

    return std::string(md);
}

std::string LedScreen::MaskKey(const std::string& key)
{
    if(key.size() <= 8)
        return "****";

    return key.substr(0, 4) + "****" + key.substr(key.size() - 4);
}

bool LedScreen::IsSuccessResponse(const std::string& response)
{
    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errors;
    std::istringstream stream(response);

    if(!Json::parseFromStream(builder, stream, &root, &errors))
    {
        std::cout << "LED response parse failed: " << errors << std::endl;
        return false;
    }

    const Json::Value message = root["message"];
    if(message.isString() && message.asString() == "ok")
        return true;

    std::cout << "LED publish rejected: " << response << std::endl;
    return false;
}

bool LedScreen::Init(const std::string& ip,
                     int port,
                     const std::string& sdkKey,
                     const std::string& sdkSecret,
                     const std::string& deviceId)
{
    ip_ = ip;
    port_ = port;
    sdkKey_ = sdkKey;
    deviceId_ = deviceId;
    sdkSecret_ = sdkSecret;

    curl_global_init(CURL_GLOBAL_DEFAULT);

    std::cout << "LED sdkKey: " << MaskKey(sdkKey_) << std::endl;

    return true;
}
#if 1
bool LedScreen::ShowText(const std::string& text)
{
    std::lock_guard<std::mutex> lock(mutex_);

    Json::Value root;
    root["method"] = "replace";
    root["id"] = deviceId_;

    Json::Value dataArray(Json::arrayValue);
    Json::Value program;

    program["name"] = "节目1";
    program["type"] = "normal";
    program["uuid"] = "A3";

    Json::Value areaArray(Json::arrayValue);
    // led 1
    Json::Value area1;

    area1["x"] = 0;
    area1["y"] = 0;
    area1["width"] = 32;
    area1["height"] = 96;

    Json::Value border1;
    border1["type"] = 0;
    border1["speed"] = 5;
    border1["effect"] = "rotate";

    area1["border"] = border1;

    Json::Value itemArray1(Json::arrayValue);
    Json::Value item1;

    item1["type"] = "text";
    item1["string"] = text;
    item1["multiLine"] = true;
    item1["PlayText"] = false;

    Json::Value font1;
    font1["name"] = "宋体";
    font1["size"] = 14;
    font1["underline"] = false;
    font1["bold"] = false;
    font1["italic"] = false;
    font1["color"] = "#ff0000";

    item1["font"] = font1;

    Json::Value effect1;
    effect1["type"] = 0;
    effect1["speed"] = 5;
    effect1["hold"] = 5000;

    item1["effect"] = effect1;

    itemArray1.append(item1);
    area1["item"] = itemArray1;

    areaArray.append(area1);

    //led 2
    Json::Value area2;

    area2["x"] = 32;
    area2["y"] = 0;
    area2["width"] = 32;
    area2["height"] = 96;

    Json::Value border2;
    border2["type"] = 0;
    border2["speed"] = 5;
    border2["effect"] = "rotate";

    area2["border"] = border2;

    Json::Value itemArray2(Json::arrayValue);
    Json::Value item2;

    item2["type"] = "text";
    item2["string"] = text;
    item2["multiLine"] = true;
    item2["PlayText"] = false;

    Json::Value font2;
    font2["name"] = "宋体";
    font2["size"] = 14;
    font2["underline"] = false;
    font2["bold"] = false;
    font2["italic"] = false;
    font2["color"] = "#ffff00";

    item2["font"] = font2;

    Json::Value effect2;
    effect2["type"] = 0;
    effect2["speed"] = 5;
    effect2["hold"] = 5000;

    item2["effect"] = effect2;

    itemArray2.append(item2);
    area2["item"] = itemArray2;

    areaArray.append(area2);

    //led3
    Json::Value area3;

    area3["x"] = 64;
    area3["y"] = 0;
    area3["width"] = 32;
    area3["height"] = 96;

    Json::Value border3;
    border3["type"] = 0;
    border3["speed"] = 5;
    border3["effect"] = "rotate";

    area3["border"] = border3;

    Json::Value itemArray3(Json::arrayValue);
    Json::Value item3;

    item3["type"] = "text";
    item3["string"] = text;
    item3["multiLine"] = true;
    item3["PlayText"] = false;

    Json::Value font3;
    font3["name"] = "宋体";
    font3["size"] = 14;
    font3["underline"] = false;
    font3["bold"] = false;
    font3["italic"] = false;
    font3["color"] = "#0000ff";

    item3["font"] = font3;

    Json::Value effect3;
    effect3["type"] = 0;
    effect3["speed"] = 5;
    effect3["hold"] = 5000;

    item3["effect"] = effect3;

    itemArray3.append(item3);
    area3["item"] = itemArray3;

    areaArray.append(area3);



    program["area"] = areaArray;

    dataArray.append(program);

    root["data"] = dataArray;

    Json::StreamWriterBuilder builder;
    builder["emitUTF8"] = true;
    builder["indentation"] = "";
    std::string body = Json::writeString(builder,root);
    std::cout << body << std::endl;
    return HttpPost(body);
}
#else
bool LedScreen::ShowText(const std::string& text)
{
    std::lock_guard<std::mutex> lock(mutex_);

    Json::Value root;
    root["method"] = "replace";
    root["id"] = deviceId_;

    Json::Value dataArray(Json::arrayValue);
    Json::Value program;

    program["name"] = "节目1";
    program["type"] = "normal";
    program["uuid"] = "A1";

    Json::Value areaArray(Json::arrayValue);
    Json::Value area;

    area["x"] = 0;
    area["y"] = 0;
    area["width"] = 32;
    area["height"] = 96;
    area["uuid"] = "B1";

    Json::Value border;
    border["type"] = 0;
    border["speed"] = 5;
    border["effect"] = "rotate";

    area["border"] = border;

    Json::Value itemArray(Json::arrayValue);
    Json::Value item;

    item["type"] = "text";
    item["string"] = text;
    item["multiLine"] = true;
    item["PlayText"] = false;
    item["uuid"] = "C1";

    Json::Value font;
    font["name"] = "宋体";
    font["size"] = 14;
    font["underline"] = false;
    font["bold"] = false;
    font["italic"] = false;
    font["color"] = "#ffff00";

    item["font"] = font;

    Json::Value effect;
    effect["type"] = 0;
    effect["speed"] = 5;
    effect["hold"] = 5000;

    item["effect"] = effect;

    itemArray.append(item);
    area["item"] = itemArray;

    areaArray.append(area);
    program["area"] = areaArray;

    dataArray.append(program);

    root["data"] = dataArray;

    Json::StreamWriterBuilder builder;
    builder["emitUTF8"] = true;
    std::string body = Json::writeString(builder,root);
    std::string body = ""
    std::cout << body << std::endl;
    return HttpPost(body);
}
#endif
bool LedScreen::HttpPost(const std::string& body)
{
     CURL* curl = curl_easy_init();
    if(!curl) return false;

    std::string url =
        "http://" + ip_ + ":" + std::to_string(port_) +
        "/api/program/";

    struct curl_slist* headers = NULL;

    std::string date = GetGMTTime();
    std::cout << "timetest: " << GetGMTTime() << std::endl;
    std::string requestId = GenerateRequestId();
    std::string raw = body + sdkKey_ + date;
    std::string sign = HmacMD5(raw, sdkSecret_);

    std::cout << "LED request url: " << url << std::endl;
    std::cout << "LED request sdkKey: " << MaskKey(sdkKey_)
              << ", date: " << date
              << ", sign: " << sign << std::endl;

    headers = curl_slist_append(headers, ("requestId: " + requestId).c_str());
    headers = curl_slist_append(headers, ("sdkKey: " + sdkKey_).c_str());
    headers = curl_slist_append(headers, ("date: " + date).c_str());
    headers = curl_slist_append(headers, ("sign: " + sign).c_str());
    headers = curl_slist_append(headers, "Content-Type: application/json");

    std::string response;
    long httpCode = 0;

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, body.size());
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 5);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

    CURLcode res = curl_easy_perform(curl);
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &httpCode);

    curl_easy_cleanup(curl);
    curl_slist_free_all(headers);

    if(res != CURLE_OK)
    {
        std::cout << "LED send failed: " << curl_easy_strerror(res) << std::endl;
        return false;
    }

    std::cout << "LED response httpCode: " << httpCode << ", body: " << response << std::endl;
    if(httpCode < 200 || httpCode >= 300)
        return false;

    return IsSuccessResponse(response);
}
