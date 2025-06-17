from django.db.models.signals import pre_save, post_save
from .models import Scanner, Cart

"""
def updateUser(sender, instance, **kwargs):
    user = instance
    if user.email != '':
        user.username = user.email
"""

def createShoppingCart(sender, instance, **kwargs):
    scanner = instance
    try:
        _ = Cart.objects.get_or_create(scanner=scanner)
    except:
        pass

post_save.connect(createShoppingCart, sender=Scanner)