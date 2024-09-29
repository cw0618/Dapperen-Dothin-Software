#include "xmlconfig.h"
#include <QtDebug>
XmlConfig::XmlConfig()
{

}
void XmlConfig::loadXmlFile(QString file_name) {
	test_xml = file_name;

}
bool XmlConfig::readXML()
{
	QFile file(test_xml);
	if (file.open(QIODevice::ReadOnly))
	{
		// 设置内容 - io对象的内容
		bool bl = m_doc.setContent(&file);
		if (bl)
		{
			file.close();
			return true;
		}
		file.close();
		return false;
	}
	else
	{
		return false;
	}
}
int XmlConfig::readXmlRegisterConfig() {

	int ret = 0;
	if (readXML())
	{
		// 拿出根节点
		QDomElement root = m_doc.documentElement();
		if (root.hasChildNodes())
		{
			if (root.hasChildNodes())
			{
				QDomElement firstElem = root.firstChildElement();
				QString sceneType = firstElem.attribute("type");
				
				QDomNodeList nodeList = firstElem.childNodes();
				xmlRegitsterBean.mRegisterBeanList.clear();
				int count = nodeList.count();
				for (int i = 0; i < nodeList.count(); ++i)
				{
					REGISTER_BEAN_DATA registerBeanItem;
					QDomElement element_item = nodeList.at(i).toElement();
					registerBeanItem.name = element_item.attribute("name");
					registerBeanItem.address = element_item.attribute("address");
					registerBeanItem.value = element_item.text();
					xmlRegitsterBean.mRegisterBeanList.append(registerBeanItem);
				}
			}

		}
	}
	return ret;
}

