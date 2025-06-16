from rest_framework.decorators import api_view, permission_classes
from rest_framework.response import Response

from base.models import Product, Cart
import urllib, base64

from base.serializer import ProductSerializer

import urllib.parse
import base64
from PIL import Image
import os, uuid
import random

@api_view(['POST'])
def getProducts(request):
    return Response("hello")
