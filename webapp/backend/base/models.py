from django.db import models
from django.conf import settings
# Create your models here.

class Robot(models.Model):
    _id = models.AutoField(primary_key=True, editable=False)
    last_seen_location = models.TextField(null=True, blank=True, default="")
    account_id = models.ForeignKey(settings.AUTH_USER_MODEL, on_delete=models.CASCADE)
    in_use = models.BooleanField(default=False)

    def __str__(self):
        return str(self.account_id.username)

class Chat(models.Model):
    _id = models.AutoField(primary_key=True, editable=False)
    robot = models.ForeignKey(Robot, on_delete=models.CASCADE)
    chat_sequence = models.IntegerField()
    chat_summary = models.TextField()

class Location(models.Model):
    _id = models.AutoField(primary_key=True, editable=False)
    robot = models.ForeignKey(Robot, on_delete=models.CASCADE)
    gps_sequence = models.IntegerField()
    gps_coordinates = models.TextField()

class Context(models.Model):
    _id = models.AutoField(primary_key=True, editable=False)
    rag = models.TextField(default='')