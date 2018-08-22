#include <pcl_text_info.h>

PCLTextInfo::PCLTextInfo()
{

}

PCLTextInfo::PCLTextInfo(std::string t, int xin, int yin)
{
    text = t;
    x = xin;
    y = yin;
}
