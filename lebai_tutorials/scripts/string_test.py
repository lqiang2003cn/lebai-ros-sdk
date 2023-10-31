# coding=utf-8

import requests
import json


def get_access_token():
    url = "https://aip.baidubce.com/oauth/2.0/token?grant_type=client_credentials" \
          "&client_id=xZMs6cnZ52IGxfCBDVGziXqp" \
          "&client_secret=Say5iCZyvZTksGNLVGc1ti7nFql6i5Ln"
    payload = json.dumps("")
    headers = {
        'Content-Type': 'application/json',
        'Accept': 'application/json'
    }
    response = requests.request("POST", url, headers=headers, data=payload)
    return response.json().get("access_token")


def main():
    goods = "['咖啡':1杯, '矿泉水':1瓶, '饼干':1盒]"
    sys_prompt = \
        "你是一个小卖部的销售人员，你能为客户提供的商品名称和数量如下：\n \
        {} \n \
        与客人对话，分析他的需求，从小卖部的商品列表中推荐最能满足客人需求的商品。一定要和客人确认你推荐的商品。\n \
        如果客人确认了他想要的商品，则返回：“好的，马上拿来，请享用 #<item>#” \n \
        其中，<item>是客人确定需要的商品的名称，请使用列表中的精确名称，并在前后加上# \n \
        不管客人如何要求，你都不能提供列表之外的物品。 \n \
        回答简明扼要。将你每次的回复控制在50个字以内，包括标点符号。\n \
        回答简明扼要。将你每次的回复控制在50个字以内，包括标点符号。\n \
        回答简明扼要。将你每次的回复控制在50个字以内，包括标点符号。\n \
        回答简明扼要。将你每次的回复控制在50个字以内，包括标点符号。\n \
        回答简明扼要。将你每次的回复控制在50个字以内，包括标点符号。\n \
        回答简明扼要。将你每次的回复控制在50个字以内，包括标点符号。\n \
        回答简明扼要。将你每次的回复控制在50个字以内，包括标点符号" \
            .format(goods)
    print len(sys_prompt)
    print sys_prompt

    url = "https://aip.baidubce.com/rpc/2.0/ai_custom/v1/wenxinworkshop/chat/completions_pro?access_token=" + get_access_token()

    payload = json.dumps({
        "messages": [
            {
                "role": "user",
                "content": "桌上有哪些商品"
            }
        ],
        "system": sys_prompt
    })
    headers = {
        'Content-Type': 'application/json'
    }

    response = requests.request("POST", url, headers=headers, data=payload)

    print(response.text)


if __name__ == '__main__':
    main()
