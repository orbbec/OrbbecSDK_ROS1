(function () {
  function getDockHost() {
    if (window.matchMedia && window.matchMedia("(max-width: 768px)").matches) {
      return document.body;
    }

    return document.querySelector(".wy-nav-side") || document.body;
  }

  function ensureDock() {
    var dock = document.querySelector("[data-docs-switch-dock]");
    var host = getDockHost();

    if (!dock) {
      dock = document.createElement("div");
      dock.className = "docs-floating-switches";
      dock.setAttribute("data-docs-switch-dock", "");
    }

    if (dock.parentNode !== host) {
      host.appendChild(dock);
    }

    return dock;
  }

  function parseConfig() {
    var element = document.getElementById("docs-version-data");
    if (!element) {
      return null;
    }

    try {
      return JSON.parse(element.textContent);
    } catch (error) {
      console.error("Failed to parse docs version configuration.", error);
      return null;
    }
  }

  function parseLocation(config) {
    var segments = window.location.pathname.split("/").filter(Boolean);
    var langIndex = segments.findIndex(function (segment) {
      return segment === "en" || segment === "zh";
    });

    if (langIndex === -1) {
      return null;
    }

    var language = segments[langIndex];
    var isVersioned = langIndex >= 2 && segments[langIndex - 2] === "versions";
    var versionSlug = isVersioned ? segments[langIndex - 1] : config.latest_slug;
    // Keep the project-pages prefix (for example /<repo>/...) when the
    // current page is served from the latest alias path /<repo>/<lang>/...
    var baseSegments = isVersioned ? segments.slice(0, langIndex - 2) : segments.slice(0, langIndex);
    var relativeSegments = segments.slice(langIndex + 1);

    if (!relativeSegments.length) {
      relativeSegments = ["index.html"];
    }

    return {
      baseSegments: baseSegments,
      language: language,
      relativeSegments: relativeSegments,
      versionSlug: versionSlug,
    };
  }

  function buildPath(baseSegments, versionSlug, latestSlug, language, relativeSegments) {
    var segments = baseSegments.slice();
    if (versionSlug !== latestSlug) {
      segments.push("versions", versionSlug);
    }
    segments.push(language);
    segments = segments.concat(relativeSegments);
    return "/" + segments.join("/");
  }

  function buildRootPath(baseSegments, versionSlug, latestSlug, language) {
    var segments = baseSegments.slice();
    if (versionSlug !== latestSlug) {
      segments.push("versions", versionSlug);
    }
    segments.push(language, "index.html");
    return "/" + segments.join("/");
  }

  function renderOptions(select, config, currentSlug) {
    select.innerHTML = "";

    config.versions.forEach(function (version) {
      var option = document.createElement("option");
      option.value = version.slug;
      option.textContent = version.label;
      option.selected = version.slug === currentSlug;
      select.appendChild(option);
    });
  }

  function canUseHeadRequest() {
    return window.location.protocol === "http:" || window.location.protocol === "https:";
  }

  function pageExists(url) {
    if (!canUseHeadRequest()) {
      return Promise.resolve(true);
    }

    return fetch(url, {
      method: "HEAD",
      cache: "no-store",
    })
      .then(function (response) {
        return response.ok;
      })
      .catch(function () {
        return false;
      });
  }

  document.addEventListener("DOMContentLoaded", function () {
    var config = parseConfig();
    var widget = document.querySelector("[data-docs-version-switch]");
    var select = document.getElementById("docs-version-select");

    if (!config || !widget || !select || !config.versions || !config.versions.length) {
      return;
    }

    function mountWidget() {
      ensureDock().appendChild(widget);
    }

    mountWidget();
    window.addEventListener("resize", mountWidget, { passive: true });

    var locationInfo = parseLocation(config);
    if (!locationInfo) {
      widget.hidden = true;
      return;
    }

    renderOptions(select, config, locationInfo.versionSlug);

    select.addEventListener("change", function (event) {
      var nextSlug = event.target.value;
      if (!nextSlug || nextSlug === locationInfo.versionSlug) {
        return;
      }

      var targetUrl = buildPath(
        locationInfo.baseSegments,
        nextSlug,
        config.latest_slug,
        locationInfo.language,
        locationInfo.relativeSegments,
      );
      var fallbackUrl = buildRootPath(
        locationInfo.baseSegments,
        nextSlug,
        config.latest_slug,
        locationInfo.language,
      );

      pageExists(targetUrl).then(function (exists) {
        window.location.href = exists ? targetUrl : fallbackUrl;
      });
    });
  });
})();
