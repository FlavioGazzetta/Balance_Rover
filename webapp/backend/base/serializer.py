from rest_framework import serializers
from .models import Product, Scanner, Cart
import json

class ProductSerializer(serializers.ModelSerializer):
    class Meta:
        model = Product
        fields = '__all__'

class ScannerSerializer(serializers.ModelSerializer):
    center = serializers.SerializerMethodField()
    radius = serializers.SerializerMethodField()
    
    class Meta:
        model = Scanner
        fields = ['center', 'radius']

    def get_center(self, obj):
        location = obj.location.split(',')
        return { 'lat': float(location[0]), 'lng': float(location[1]) }
    
    def get_radius(self, obj):
        location = obj.location.split(',')
        return float(location[2])

class CartSerializer(serializers.ModelSerializer):
    class Meta:
        model = Cart
        fields = '__all__'