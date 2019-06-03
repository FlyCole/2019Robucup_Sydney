# -*- coding: utf-8 -*-
"""
    Author: Yifei Ren
    Name: human_feature_detect
    Version: 1.0
    Date: 25/05/2019
    Description: Detect the clothing of hunman and print.
    Note: When you use this function, annotate the main function and create
          the class object out of the file.
"""
import urllib, urllib2, sys
import ssl
import cv2
import base64
import json



def feature(img_name):
    max_rectangle_geder = "none"
    context = ssl._create_unverified_context()
    # client_id 为官网获取的AK， client_secret 为官网获取的SK
    host = 'https://aip.baidubce.com/oauth/2.0/token?grant_type=client_credentials&client_id=C4Ek0ufwNB09LYxTm24Q17V8&client_secret=te51c9fpdzhGhT5rmbxgVTgipau7qZrO'
    request = urllib2.Request(host)
    request.add_header('Content-Type', 'application/json; charset=UTF-8')
    response = urllib2.urlopen(request, context=context)
    content1 = response.read()
    img = cv2.imread(img_name)
    request_url = "https://aip.baidubce.com/rest/2.0/image-classify/v1/body_attr"
    f = open(img_name, 'rb')
    image = base64.b64encode(f.read())
    image64 = str(image).encode("utf-8")
    params = {"image":''+image64+'',"image_type":"BASE64",
              "type":"gender,age,lower_wear,upper_wear,upper_color,lower_color"}
    params = urllib.urlencode(params).encode("utf-8")
    access_token = content1.split("\"")[13]
    request_url = request_url + "?access_token=" + access_token
    request = urllib2.Request(url=request_url, data=params)
    request.add_header('Content-Type', 'application/x-www-form-urlencoded')
    response = urllib2.urlopen(request, context=context)
    content = response.read()
    dict_info = json.loads(content)
    print "================================="
    print content
    print "================================="
    male_num = 0
    female_num = 0
    try:
        people_list = dict_info["person_info"]
    except:
        return 0, 0, "none"
    for i in range(len(people_list)):
        left = int(people_list[i]["location"]["left"])
        top = int(people_list[i]["location"]["top"])
        right = int(people_list[i]["location"]["left"] + people_list[i]["location"]["width"])
        bottom = int(people_list[i]["location"]["top"] + people_list[i]["location"]["height"])

        cv2.rectangle(img, (left, top), (right, bottom), (0, 0, 255), 2)
        # if people_list[i]["attributes"]["age"]["name"].encode("UTF-8") == "青年":
        #     print "Person", i + 1, "is a", "teenager"

        lower_color = people_list[i]["attributes"]["lower_color"]["name"].encode("UTF-8")
        lower_color = trans_color(lower_color)
        upper_color = people_list[i]["attributes"]["upper_color"]["name"].encode("UTF-8")
        upper_color = trans_color(upper_color)

        lower_wear = people_list[i]["attributes"]["lower_wear"]["name"].encode("UTF-8")
        lower_wear = trans_wear(lower_wear)
        upper_wear = people_list[i]["attributes"]["upper_wear"]["name"].encode("UTF-8")
        upper_wear = trans_wear(upper_wear)


        print "The person", i + 1, "is wearing", lower_color, lower_wear
        print "The person", i + 1, "is wearing", upper_color, upper_wear

    cv2.imwrite("./feature_result.jpg", img)
    return male_num, female_num, max_rectangle_geder


def trans_color(color):
    """
    Translation of color from Chinese to English
    :param color:
    :return:color:
    """
    if color == "白":
        color = "white"
    elif color == "蓝":
        color = "blue"
    elif color == "绿":
        color = "green"
    elif color == "黑":
        color = "black"
    elif color == "红":
        color = "red"
    elif color == "黄":
        color = "yellow"
    return color

def trans_wear(wear):
    """
    Translation of clothing from Chinese to English
    :param wear:
    :return:
    """
    if wear == "短裤":
        wear = "shorts"
    elif wear == "短袖":
        wear = "short sleeve"
    elif wear == "长裤":
        wear = "trousers"
    elif wear == "长袖":
        wear = "long sleeve"
    elif wear == "外套":
        wear = "coat"
    elif wear == "夹克":
        wear = "jacket"
    elif wear == "不确定":
        wear = "cloth, the type of which is not sure"
    return wear


if __name__ == '__main__':
    feature("./person_image.jpg")
