# Custom User Model Setup

## Overview
This document explains the custom user model implementation and the changes made to set up a custom user authentication system instead of using Django's default `User` model.

## Why a Custom User Model?
Django's default `User` model is generic and works for basic use cases. By creating a custom user model, we can:
- Add custom fields specific to our application
- Make certain fields required (like email)
- Customize the authentication behavior
- Have more control over user data

## Changes Made

### 1. Models (`accounts/models.py`)

We created a `CustomUser` model that extends Django's `AbstractUser`:

```python
from django.contrib.auth.models import AbstractUser

class CustomUser(AbstractUser):
    email = models.EmailField(unique=True, blank=False)
```

**Key Points:**
- `AbstractUser` provides all the default user functionality (username, password, first_name, last_name, etc.)
- We made the `email` field **required** (`blank=False`) and **unique** to ensure each user has a unique email address
- This model replaces Django's default `User` model

### 2. Forms (`accounts/forms.py`)

We created custom forms that work with our `CustomUser` model:

#### CustomUserCreationForm
```python
class CustomUserCreationForm(UserCreationForm):
    class Meta(UserCreationForm.Meta):
        model = CustomUser
        # Add 'email' to the fields tuple to make it a required field in the user creation form
        # This ensures users must provide an email when registering
        fields = UserCreationForm.Meta.fields + ('email',)
```

**What this does:**
- Extends Django's built-in `UserCreationForm`
- Sets the model to our `CustomUser`
- **Adds email to the form fields** - This makes email a required field when creating a user
- Users must provide email during registration

#### CustomUserChangeForm
```python
class CustomUserChangeForm(UserChangeForm):
    class Meta(UserChangeForm.Meta):
        model = CustomUser
        fields = UserChangeForm.Meta.fields
```

**What this does:**
- Extends Django's built-in `UserChangeForm`
- Used when updating an existing user's information
- Works with our `CustomUser` model

### 3. Admin Interface (`accounts/admin.py`)

We registered the custom user model with Django's admin:

```python
class CustomUserAdmin(UserAdmin):
    add_form = CustomUserCreationForm      # Form used when creating a new user
    form = CustomUserChangeForm            # Form used when editing an existing user
    model = CustomUser                     # The model to manage
    # list_display: Controls which fields are displayed in the user list view in the admin interface
    # Shows username and email columns in the user list
    list_display = ['username', 'email']
    # fieldsets: Organizes fields into sections when viewing/editing an existing user
    # Uses the default UserAdmin fieldsets structure
    fieldsets = UserAdmin.fieldsets
    # add_fieldsets: Organizes fields into sections when creating a new user
    # Extends the default fieldsets to include the email field in the user creation form
    add_fieldsets = UserAdmin.add_fieldsets + (
        (None, {'fields': ('email',)}),
    )
```

**What this does:**
- Creates a custom admin class for managing users
- Uses our custom forms instead of Django's default forms
- `list_display` - Shows username and email columns in the user list view
- `fieldsets` - Organizes fields into sections when editing an existing user
- `add_fieldsets` - Organizes fields when creating a new user, with email included
- This appears in the Django admin dashboard at `/admin/`
- Administrators can create, edit, and delete users through the admin interface

### 4. Database Configuration (`config/settings.py`)

In the Django settings file, we specify the custom user model:

```python
AUTH_USER_MODEL = 'accounts.CustomUser'
```

**What this does:**
- Tells Django to use our `CustomUser` model instead of the default `User` model
- Must be set before running migrations
- All foreign key references to users should use this custom model

### 5. Migrations

After making these changes, we created migrations for the accounts app:

```bash
python manage.py makemigrations accounts
```

This created `accounts/migrations/0001_initial.py` which contains the database schema for:
- Creating the `CustomUser` model
- Setting up all the fields and constraints

To apply the migrations to the database:
```bash
python manage.py migrate accounts
```

### 6. Views (`accounts/views.py`)

We created views for user authentication:

```python
from django.shortcuts import render
from django.urls import reverse_lazy
from django.views import generic
from django.contrib.auth.views import LoginView, LogoutView
from .forms import CustomUserCreationForm

# Sign Up View - Creates a new user
class SignUpView(generic.CreateView):
    form_class = CustomUserCreationForm
    success_url = reverse_lazy('login')  # Redirect to login page after successful signup
    template_name = 'registration/signup.html'

# Login View - User logs into their account
class CustomLoginView(LoginView):
    template_name = 'registration/login.html'
    success_url = reverse_lazy('home')

# Logout View - User logs out (built-in Django view)
# Redirects to home page after logout (configured in settings)
```

**What this does:**
- `SignUpView` - Handles user registration, uses `CustomUserCreationForm`, requires email
- `LoginView` - Handles user login, redirects to home page on success
- `LogoutView` - Django's built-in logout handler, requires POST request for security
- After logout, users are redirected to the home page

### 7. URL Configuration (`accounts/urls.py` and `config/urls.py`)

We created URL patterns for authentication:

```python
# accounts/urls.py
from django.urls import path
from .views import SignUpView, CustomLoginView
from django.contrib.auth.views import LogoutView

urlpatterns = [
    path('signup/', SignUpView.as_view(), name='signup'),
    path('login/', CustomLoginView.as_view(), name='login'),
    path('logout/', LogoutView.as_view(http_method_names=['get', 'post']), name='logout'),
]
```

```python
# config/urls.py
from django.contrib import admin
from django.urls import path, include
from django.views.generic import TemplateView

urlpatterns = [
    path('admin/', admin.site.urls),
    path('accounts/', include('accounts.urls')),
    path('', TemplateView.as_view(template_name='home.html'), name='home'),
]
```

**What this does:**
- Maps SignUp, Login, and Logout views to URLs
- `logout/` accepts both GET and POST for flexibility (normally only POST is required)
- Home page is set as the root URL

### 8. Settings Configuration (`config/settings.py`)

We configured the authentication settings:

```python
# Custom User Model
AUTH_USER_MODEL = 'accounts.CustomUser'

# Login/Logout Redirects
LOGIN_REDIRECT_URL = 'home'      # After successful login, redirect to home
LOGOUT_REDIRECT_URL = 'home'     # After logout, redirect to home
LOGIN_URL = 'login'              # Where to send unauthorized users

# Templates
TEMPLATES = [{
    'DIRS': [str(BASE_DIR.joinpath('templates'))],
    # ...
}]
```

**What this does:**
- Specifies the custom user model
- Sets redirect URLs for login/logout
- Tells Django where to find templates
- `LOGIN_URL` redirects unauthenticated users to the login page

### 9. Templates

We created three main templates:

```html
<!-- templates/base.html -->
<!-- Base template for all pages -->
{% if user.is_authenticated %}
    <p>Welcome, {{ user.username }}!</p>
    <form method="post" action="{% url 'logout' %}" style="display: inline;">
        {% csrf_token %}
        <button type="submit">Logout</button>
    </form>
{% else %}
    <a href="{% url 'login' %}">Login</a>
    <a href="{% url 'signup' %}">Sign Up</a>
{% endif %}

<!-- templates/registration/signup.html -->
<!-- User registration form -->
<form method="post">{% csrf_token %}
    {{ form.as_p }}
    <button type="submit">Sign Up</button>
</form>

<!-- templates/registration/login.html -->
<!-- User login form -->
<form method="post">{% csrf_token %}
    {{ form.as_p }}
    <button type="submit">Login</button>
</form>

<!-- templates/home.html -->
<!-- Home page shown to all users -->
{% extends 'base.html' %}
```

**What this does:**
- `base.html` - Shows login/signup links if not authenticated, logout button if authenticated
- `signup.html` - User registration form
- `login.html` - User login form
- `home.html` - Home page that extends base template
- Logout form uses POST method for security (CSRF protection)

## Summary of the Flow

1. **User Registration** → `/accounts/signup/` → Uses `SignUpView` and `CustomUserCreationForm` → Requires email → Redirects to login
2. **User Login** → `/accounts/login/` → Uses `LoginView` → Redirects to home on success
3. **User Logout** → `/accounts/logout/` → Uses `LogoutView` (POST request) → Redirects to home
4. **Admin Panel** → `/admin/` → Uses `CustomUserAdmin` class
5. **Database** → All users stored in `CustomUser` model
6. **Authentication** → Protected by Django's auth middleware, uses custom user model

## Benefits of This Setup

✓ Email is required for all users
✓ Email must be unique (no duplicate emails)
✓ Full authentication workflow (signup, login, logout)
✓ Secure logout with CSRF protection
✓ Easy to add more custom fields in the future
✓ Full control over user creation and management
✓ Works seamlessly with Django's authentication system
✓ Admin interface automatically supports our custom user model
✓ Protected pages redirect unauthorized users to login
