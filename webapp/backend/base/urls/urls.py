from django.urls import path
from base.views import views as views
# from rest_framework_simplejwt.views import TokenObtainPairView

urlpatterns = [
    path('location/', views.getCoords, name='coords'),
    path('memory/', views.getChatMemory, name='chat-memory'),
    path('update/', views.setChatMemory, name='latest'),
    path('all/', views.getAllCoord, name='all'),
    path('clear/', views.clearChatMemory, name='clear'),
]