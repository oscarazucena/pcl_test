#ifndef PCLTEXTINFO_H
#define PCLTEXTINFO_H

#include <string>

struct PCLTextInfo
{
    PCLTextInfo();
    PCLTextInfo(std::string t, int xin, int yin);
    int x;
    int y;
    std::string text;
};
#endif // PCLTEXTINFO_H

