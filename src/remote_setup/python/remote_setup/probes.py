"""Read-only probes, run in a disposable child with an overall process deadline."""
import asyncio
import json
import logging
import ssl
import sys
from urllib import error, request

from .config import validate

EXCHANGE = 'catarm.events'


async def broker(settings, connect=None):
    connection = None
    stage = 'login'
    try:
        if connect is None:
            import aio_pika
            connect = aio_pika.connect
        connection = await connect(
            host=settings['SERVER_HOST'], port=int(settings['AMQP_PORT']),
            login=settings['AMQP_USERNAME'], password=settings['AMQP_PASSWORD'],
            virtualhost=settings['AMQP_VHOST'], ssl=settings['AMQP_TLS'] == 'true',
            ssl_context=ssl.create_default_context() if settings['AMQP_TLS'] == 'true' else None,
            timeout=settings.timeout, client_properties={'connection_name': 'PicknPlace remote_setup test'})
        stage = 'exchange'
        channel = await connection.channel(publisher_confirms=False)
        await channel.declare_exchange(EXCHANGE, passive=True, timeout=settings.timeout)
        return {'check': 'RabbitMQ', 'ok': True,
                'message': 'Login/vhost accepted; catarm.events exists. Connection closed after test.'}
    except ImportError:
        return {'check': 'RabbitMQ', 'ok': False,
                'message': 'AMQP dependency missing; install remote_setup requirements.'}
    except Exception as exc:
        # Never expose broker reply text, URLs, credentials or arbitrary exception repr.
        code = getattr(exc, 'reply_code', None)
        category = ('timeout' if isinstance(exc, (TimeoutError, asyncio.TimeoutError)) else
                    'TLS certificate/handshake failure' if isinstance(exc, ssl.SSLError) else
                    'authentication/vhost rejected' if type(exc).__name__ in
                    ('ProbableAuthenticationError', 'AuthenticationError', 'ConnectionNotAllowed') else
                    f'broker refusal {code}' if isinstance(code, int) else
                    'connection or protocol failure')
        detail = 'Check host/port, TLS, credentials and vhost.' if stage == 'login' else \
            'Check that catarm.events exists and this account can inspect it.'
        return {'check': 'RabbitMQ', 'ok': False, 'message': f'{stage}: {category}. {detail}'}
    finally:
        if connection is not None:
            try:
                await asyncio.wait_for(connection.close(), timeout=2.)
            except Exception:
                pass  # The disposable child exits; no reconnect or retained socket.


class NoRedirect(request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


def http_health(settings, opener=None):
    try:
        # Ignore workstation proxy settings; contact the explicitly configured server.
        opener = opener or request.build_opener(request.ProxyHandler({}), NoRedirect())
        url = settings['CONVEYOR_API_URL'].rstrip('/') + '/api/health'
        with opener.open(request.Request(url, headers={'Accept': 'application/json'}),
                         timeout=settings.timeout) as response:
            if response.status != 200:
                raise ValueError('Expected HTTP 200')
            data = response.read(65537)
            if len(data) > 65536:
                raise ValueError('Health response exceeds 64 KiB')
        body = json.loads(data)
        if not isinstance(body, dict) or body.get('status') != 'ok' or \
                body.get('service') != 'conveyor-server':
            raise ValueError('Health response is not a healthy CATARM conveyor-server')
        if body.get('belt_id') != settings['BELT_ID']:
            raise ValueError('Server belt_id does not match the configured belt')
        return {'check': 'Conveyor API', 'ok': True,
                'message': 'Healthy conveyor-server; belt identity matches.'}
    except error.HTTPError as exc:
        message = f'HTTP {exc.code}; check the conveyor API base URL (redirects are refused).'
    except (json.JSONDecodeError, UnicodeError):
        message = 'Health endpoint returned invalid JSON.'
    except ValueError as exc:
        message = str(exc)  # Only the fixed messages above; no remote response text.
    except Exception:
        message = 'Connection/TLS/timeout failure; check the conveyor API URL and certificate.'
    return {'check': 'Conveyor API', 'ok': False, 'message': message}


def main():
    # Libraries can log credential-bearing protocol replies; emit only sanitized results.
    logging.disable(logging.CRITICAL)
    try:
        settings = validate(json.loads(sys.stdin.read(65536)))

        async def amqp_check():
            return await asyncio.wait_for(broker(settings), settings.timeout + 3.)
        try:
            amqp = asyncio.run(amqp_check())
        except (TimeoutError, asyncio.TimeoutError):
            amqp = {'check': 'RabbitMQ', 'ok': False, 'message': 'Connection test timed out.'}
        results = [amqp, http_health(settings)]
        print(json.dumps({'ok': all(r['ok'] for r in results), 'results': results}), flush=True)
        return 0 if all(r['ok'] for r in results) else 1
    except Exception:
        print(json.dumps({'ok': False, 'results': [
            {'check': 'Settings', 'ok': False, 'message': 'Cannot load valid saved remote settings.'}]}))
        return 2


if __name__ == '__main__':
    sys.exit(main())
