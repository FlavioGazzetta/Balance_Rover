from rest_framework.decorators import api_view, permission_classes
from rest_framework.permissions import IsAuthenticated, IsAdminUser
from rest_framework.response import Response
from django.http import FileResponse
from django.contrib.auth import authenticate 

from base.models import Robot, Location, Chat
from base.serializer import RobotSerializer, LocationSerializer, ChatSerializer
from rest_framework.authtoken.models import Token
import requests, os
from openai import OpenAI
from typing import List
from itertools import groupby
from operator import itemgetter

KEY = os.environ.get("KEY")

@api_view(['POST'])
def getCoords(request):
    data = request.data
    user = authenticate(request, username="robot1", password="12345678lol")
    robot = Robot.objects.get(account_id=user)
    try:
        lat, lng = request.data.get('lat'), request.data.get('lng')
        r = requests.get(f"https://maps.googleapis.com/maps/api/geocode/json?latlng={lat},{lng}&key={KEY}").json()["results"][0]
        w = requests.get(f"https://weather.googleapis.com/v1/currentConditions:lookup?key={KEY}&location.latitude={lat}&location.longitude={lng}").json()
        extracted_info = {"location": r["formatted_address"], 
                        "temperature": w["temperature"]["degrees"], 
                        "feels_like": w["feelsLikeTemperature"]["degrees"],
                        "humidity": w["relativeHumidity"]}
        new_sequence = Location.objects.filter(robot=robot).count() + 1
        _ = Location.objects.create(
            robot=robot,
            gps_sequence=new_sequence,
            gps_coordinates=f"{lat},{lng}"
        )
        return Response(extracted_info)
    except:
        extracted_info = {"location": "South Kensington, London", 
                        "temperature": 0, 
                        "feels_like": 0,
                        "humidity": 0}
        return Response(extracted_info)

@api_view(['GET'])
def getAllCoord(request):
    qs = (
        Location.objects
        .values('robot_id', 'gps_sequence', 'gps_coordinates')
        .order_by('robot_id', 'gps_sequence')
    )
    all_paths: List[List[List[float]]] = []
    for robot_id, rows in groupby(qs, key=itemgetter('robot_id')):
        robot_path: List[List[float]] = []
        for row in rows:
            try:
                lat_str, lng_str = row['gps_coordinates'].split(',', 1)
                robot_path.append([float(lat_str), float(lng_str)])
            except (ValueError, AttributeError):
                continue
        if robot_path:
            all_paths.append(robot_path)

    return Response(all_paths)


@api_view(['GET'])
def getChatMemory(request):
    user = authenticate(request, username="robot1", password="12345678lol")
    robot = Robot.objects.get(account_id=user)
    rows = (
        Chat.objects
        .filter(robot=robot)
        .values_list("chat_sequence", "chat_summary")
        .order_by("chat_sequence")
    )
    parts = [f"{seq}: {summary}" for seq, summary in rows]
    return Response("\n".join(parts))
    
@api_view(['GET'])
def clearChatMemory(request):
    user = authenticate(request, username="robot1", password="12345678lol")
    robot = Robot.objects.get(account_id=user)
    Chat.objects.filter(robot=robot).delete()
    return Response("Success")

@api_view(['POST'])
def setChatMemory(request):
    user = authenticate(request, username="robot1", password="12345678lol")
    robot = Robot.objects.get(account_id=user)
    req = request.data.get('prompt')
    res = request.data.get('response')
    client = OpenAI(
        api_key = os.environ.get("OPENAI_API_KEY"),
    )
    response = client.responses.create(
        model="gpt-4.1-nano",
        input = f"""
        Summarize the following conversation in 1-2 short sentences each.

        User: {req}

        Assistant: {res}

        Return your answer in this format, user_summary starts with (User asked about ...) and assistant_summary starts with (You ...):
        user_summary → assistant_summary
        """
    )
    new_sequence = Chat.objects.filter(robot=robot).count() + 1
    Chat.objects.create(
        robot=robot,
        chat_sequence=new_sequence,
        chat_summary=response.output_text
    )
    return Response("Success")
