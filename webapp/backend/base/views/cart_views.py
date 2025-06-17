from rest_framework.decorators import api_view, permission_classes
from rest_framework.permissions import IsAuthenticated, IsAdminUser
from rest_framework.response import Response
from django.http import FileResponse

from base.models import Cart, Scanner, Product
from base.serializer import ScannerSerializer, CartSerializer
import random
import requests, os
from pathlib import Path

KEY = os.environ.get("KEY")
LAT = "51.498"
LNG = "-0.176"

@api_view(['GET'])
def getCart(request):
    return Response("Success", status=200)

@api_view(['POST'])
def getCoords(request):
    data = request.data
    if data['id'] != 'imperial.ac.uk':
        return Response("Success", status=200)
    try:
        lat, lng = data['lat'], data['lng']
        r = requests.get(f"https://maps.googleapis.com/maps/api/geocode/json?latlng={lat},{lng}&key={KEY}").json()["results"][0]
        w = requests.get(f"https://weather.googleapis.com/v1/currentConditions:lookup?key={KEY}&location.latitude={lat}&location.longitude={lng}").json()
        """
        coords = Scanner.objects.all()
        serializer = ScannerSerializer(coords, many=True)
        """
        extracted_info = {"location": r["formatted_address"], 
                        "temperature": w["temperature"]["degrees"], 
                        "feels_like": w["feelsLikeTemperature"]["degrees"],
                        "humidity": w["relativeHumidity"]}
        return Response(extracted_info)
    except:
        w = requests.get(f"https://weather.googleapis.com/v1/currentConditions:lookup?key={KEY}&location.latitude={LAT}&location.longitude={LNG}").json()
        extracted_info = {"location": "South Kensington, London", 
                        "temperature": w["temperature"]["degrees"], 
                        "feels_like": w["feelsLikeTemperature"]["degrees"],
                        "humidity": w["relativeHumidity"]}
        return Response(extracted_info)


@api_view(['GET'])
def getChatMemory(request):
    text = """Previously in this chat:
    1. User asked about dormitory options → You described available dorms and amenities.
    2. User asked for directions to the library → You gave walking directions from the main gate.
    3. User asked about dining hall hours → You listed breakfast, lunch, and dinner times.
    """
    return Response(text, status=200)
    
@api_view(['POST'])
def getLatestTrade(request):
    id = request.data['id']
    cart = Cart.objects.get(_id=id)
    serializer = CartSerializer(cart, many=False)
    return Response(serializer.data)
