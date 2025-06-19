from django.contrib import admin
from .models import Robot, Location, Chat, Context
# Register your models here.
admin.site.register(Robot)
admin.site.register(Location)
admin.site.register(Chat)
admin.site.register(Context)