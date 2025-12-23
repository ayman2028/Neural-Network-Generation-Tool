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
- **Purpose:** Display detailed information about a single network and provide download functionality
- **URL Parameter:** `pk` (network ID)
- **HTTP Methods:**
  - **GET**: Display network configuration page with download button (if files exist)
  - **POST**: Download generated network files as ZIP
- **File:** `networks/views.py`
- **Download Functionality:**
  - POST method downloads generated ZIP file
  - Filename format: `network_{name}_{mode}_{input_size}.zip` (spaces replaced with underscores)
  - Validates that `generated_files_path` is set (returns 404 if empty)
  - Validates that file exists on disk (returns 404 if missing)
  - Returns FileResponse with proper Content-Type and Content-Disposition headers
  - Handles errors with helpful user-facing messages

### 3. NetworkCreateView
- **Type:** `CreateView`
- **Model:** `NetworkConfig`
- **Template:** `networks/network_form.html`
- **Form Class:** `NetworkConfigForm` (custom form with B field and parameter population)
- **Form Fields:** `name`, `description`, `mode`, `input_size`, `T`, `R`, `B`
- **Purpose:** Display form to create a new network and save to database
- **Additional Functionality:** 
  - Automatically calls `generator.generate_network()` after form submission
  - Populates mode-specific parameters **only on successful generation**:
    - **Mode 1/2**: Sets `output_size` (M) = input_size, `P` = 1 for Mode 1
    - **Mode 3**: Sets `layer_sizes` = [input_size, input_size, 1]
  - Stores path to generated ZIP file in `generated_files_path`
- **Generation Flow:** Form submission → Save to DB → Generate files → Populate parameters → Save updated record → Redirect to detail view
- **Error Handling:** Generation errors are logged but don't prevent network creation. Parameters remain empty/null if generation fails (no partial data).
- **B Parameter:** Default value 10 (multiplier budget for Mode 3 optimization). Users can customize.
- **Redirect:** Redirects to `network_detail` with the newly created network's ID
- **File:** `networks/views.py`
- **Status:** ✓ Fully implemented with automatic file generation and parameter population
- **Tests:** ✓ 12 dedicated tests covering all functionality

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

### Form Input Fields
- `name` - Network name (required)
- `description` - Optional description
- `mode` - Generation mode (1, 2, or 3) (required)
- `input_size` - Input dimension (N) (required)
- `T` - Bit width (required)
- `R` - ReLU activation (boolean)
- `B` - Multiplier budget (default: 10, used for Mode 3 optimization)

### Auto-Populated Fields (after successful generation)
- `output_size` - Output dimension (M) - Mode 1/2 only. Set to input_size.
- `P` - Parallelism factor - Mode 1/2 only. Set to 1 for Mode 1.
- `layer_sizes` - List of layer dimensions [M1, M2, M3] - Mode 3 only.
- `generated_files_path` - Path to generated ZIP file.

### Metadata
- `created_at` - Auto-populated timestamp
- `updated_at` - Auto-populated timestamp

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
