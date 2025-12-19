---
layout: default
title: Indice Automatico
---

# Tutti gli Appunti

<ul>
  {% for page in site.html_pages %}
    {% if page.url != "/" and page.url != "/404.html" and page.path contains 'Davide/Theory' %}
      <li>
        <a href="{{ page.url | relative_url }}">
          {{ page.title | default: page.url }}
        </a>
      </li>
    {% endif %}
  {% endfor %}
</ul>
