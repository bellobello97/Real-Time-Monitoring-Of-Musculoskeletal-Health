var arduinoCookieSolution = (function (exports) {
  'use strict';

  function loadJS(url, {async = false}) {
    return new Promise((resolve, reject) => {
      // Create a new script element
      const script = window.document.createElement('script');
      script.src = url;
      script.onload = resolve;
      script.async = async;

      // Inject the script into the DOM
      const ref = window.document.getElementsByTagName('script')[0];
      ref.parentNode.insertBefore(script, ref);
      setTimeout(reject, 2000);
    });
  }

  function initCookieConsent(coppa) {
    let lang = 'en';

    let content = `<div id="iubenda-cs-title">We use cookies &#127850;</div><div id="iubenda-cs-description">Our websites use cookies (also from third parties) for functional and analytical purposes. You can adjust this in <a class="iubenda-cs-customize-btn">Cookie Settings</a> or learn more by reading our %{cookie_policy_link}.</div>`;

    if (navigator?.language === 'it-IT') {
      lang = 'it';
      content = `<div id="iubenda-cs-title">Usiamo i cookies &#127850;</div><div id="iubenda-cs-description">I nostri siti usano cookie (anche di terze parti) per fini funzionali e di analisi. Puoi regolare queste impostazioni nelle <a class="iubenda-cs-customize-btn">Impostazioni di tracciamento</a> o saperne di pi&ugrave; leggendo la %{cookie_policy_link}.</div>`;
    }

    window._iub = window._iub || [];
    window._iub.csConfiguration = {
      googleConsentMode: 'template',
      askConsentAtCookiePolicyUpdate: true,
      ccpaAcknowledgeOnDisplay: false,
      ccpaApplies: false,
      ccpaNoticeDisplay: false,
      consentOnContinuedBrowsing: false,
      cookiePolicyId: 11225532,
      countryDetection: true,
      enableCcpa: false,
      floatingPreferencesButtonDisplay: false,
      startOnDomReady: true,
      lang,
      // localConsentDomain: 'arduino.cc',
      perPurposeConsent: true,
      purposes: coppa ? '1, 4' : '1, 4, 5',
      siteId: 2023027,
      whitelabel: true,
      cookiePolicyUrl: lang === 'it' ? 'https://www.arduino.cc/' + lang + '/cookie-policy' : 'https://www.arduino.cc/en/cookie-policy',
      banner: {
        applyStyles: false,
        content,
        rejectButtonDisplay: true,
        rejectButtonCaption: lang === 'it' ? 'SOLO NECESSARI' : 'ONLY REQUIRED',
        position: 'float-bottom-left',
        acceptButtonDisplay: true,
        acceptButtonCaption: lang === 'it' ? 'ACCETTA TUTTI' : 'ACCEPT ALL',
        backgroundOverlay: false,
        brandBackgroundColor: 'black'
      },
      callback: {
        // push events to google tag manager to enable the firing of specific tags according to the preference given by the user
        onPreferenceExpressedOrNotNeeded: preference => {
          const dataLayer = window.dataLayer || [];
          dataLayer.push({
            // eslint-disable-next-line camelcase
            iubenda_ccpa_opted_out: window._iub.cs.api.isCcpaOptedOut()
          });
          if (preference) {
            if (preference.consent === true) {
              dataLayer.push({
                event: 'iubenda_consent_given'
              });
            } else if (preference.consent === false) {
              dataLayer.push({
                event: 'iubenda_consent_rejected'
              });
            } else if (preference.purposes) {
              for (const purposeId in preference.purposes) {
                if (preference.purposes[purposeId]) {
                  dataLayer.push({
                    event: 'iubenda_consent_given_purpose_' + purposeId
                  });
                }
              }
            }
          } else {
            dataLayer.push({
              event: 'iubenda_preference_not_needed'
            });
          }
        }
      }
    };

    return loadJS(
      'https://cdn.arduino.cc/header-footer/prod/iubenda-1463.js',
      {async: true}
    );
  }

  exports.initCookieConsent = initCookieConsent;

  Object.defineProperty(exports, '__esModule', { value: true });

  return exports;

}({}));
