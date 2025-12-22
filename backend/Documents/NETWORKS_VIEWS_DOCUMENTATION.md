# Networks App - Views & URLs Documentation

## Overview
This document tracks all views, URL patterns, templates, and redirect flows for the Networks app.

---

## Views

### 1. NetworkListView
- **Type:** `ListView`
- **Model:** `NetworkConfig`
- **Template:** `networks/network_list.html`
- **Context Variable:** `networks`
- **Purpose:** Display all networks in the database
- **File:** `networks/views.py`

### 2. NetworkDetailView
- **Type:** `DetailView`
- **Model:** `NetworkConfig`
- **Template:** `networks/network_detail.html`
- **Context Variable:** `network`
- **Purpose:** Display detailed information about a single network
- **URL Parameter:** `pk` (network ID)
- **File:** `networks/views.py`

### 3. NetworkCreateView
- **Type:** `CreateView`
- **Model:** `NetworkConfig`
- **Template:** `networks/network_form.html`
- **Form Fields:** `name`, `description`, `mode`, `input_size`, `T`, `R`
- **Purpose:** Display form to create a new network and save to database
- **Redirect:** Redirects to `network_detail` with the newly created network's ID
- **File:** `networks/views.py`

### 4. NetworkDownloadView (PENDING)
- **Type:** `View`
- **Model:** `NetworkConfig`
- **Purpose:** Download generated network files as ZIP via AJAX
- **URL Parameter:** `pk` (network ID)
- **Returns:** FileResponse with ZIP file
- **Status:** Not yet implemented
- **File:** `networks/views.py`

### 5. NetworkDeleteView (PENDING)
- **Type:** `DeleteView`
- **Model:** `NetworkConfig`
- **Template:** `networks/network_confirm_delete.html`
- **Purpose:** Delete a network from the database with confirmation
- **URL Parameter:** `pk` (network ID)
- **Redirect:** Back to `network_list` after deletion
- **Status:** Not yet implemented
- **File:** `networks/views.py`

---

## URL Patterns

### Base Path
`/networks/`

| Endpoint | Method | Name | View | Template | Description |
|----------|--------|------|------|----------|-------------|
| `/networks/` | GET | `list` | `network_list` | `network_list.html` | List all networks |
| `/networks/create/` | GET/POST | `create` | `network_create` | `network_form.html` | Create new network |
| `/networks/<int:pk>/` | GET | `detail` | `network_detail` | `network_detail.html` | View network details |
| `/networks/<int:pk>/download/` | GET | `download` | `network_download` | - | Download network files (PENDING) |
| `/networks/<int:pk>/delete/` | GET/POST | `delete` | `network_delete` | - | Delete network (PENDING) |

---

## Redirect Flow

```
User visits /networks/
    ↓
NetworkListView (displays all networks)
    ↓
User clicks "Create New Network"
    ↓
/networks/create/
    ↓
NetworkCreateView - displays form (GET)
    ↓
User submits form (POST)
    ↓
Data saved to database
    ↓
Redirects to /networks/<pk>/ (detail view)
    ↓
NetworkDetailView (displays newly created network)
    ↓
User can click "Back to Networks" to return to list
```

---

## Template Files

| Template | Location | View | Purpose |
|----------|----------|------|---------|
| `network_list.html` | `templates/networks/network_list.html` | NetworkListView | Displays list of all networks |
| `network_detail.html` | `templates/networks/network_detail.html` | NetworkDetailView | Displays single network details |
| `network_form.html` | `templates/networks/network_form.html` | NetworkCreateView | Form for creating networks |

All templates extend `base.html`

---

## Model Fields Used

From `NetworkConfig` model:
- `name` - Network name
- `description` - Optional description
- `mode` - Generation mode (1, 2, or 3)
- `input_size` - Input dimension
- `T` - Bit width
- `R` - ReLU activation (boolean)
- `created_at` - Auto-populated timestamp
- `updated_at` - Auto-populated timestamp
- `generated_files_path` - Optional path to generated files

---

## Future Implementation Notes

### NetworkDownloadView
- Needs to retrieve network config by `pk`
- Generate or retrieve generated files
- Compress files as ZIP
- Return file download response

### NetworkDeleteView
- Django's built-in `DeleteView` recommended
- Should redirect back to `network_list` after deletion
- May need confirmation template

---

## URL Reversal Examples

In templates, use:
```django
{% url 'networks:list' %}           → /networks/
{% url 'networks:create' %}         → /networks/create/
{% url 'networks:detail' pk=1 %}    → /networks/1/
{% url 'networks:download' pk=1 %}  → /networks/1/download/ (pending)
{% url 'networks:delete' pk=1 %}    → /networks/1/delete/ (pending)
```

In Python views:
```python
from django.urls import reverse_lazy
reverse_lazy('networks:list')
reverse_lazy('networks:detail', kwargs={'pk': self.object.pk})
```
